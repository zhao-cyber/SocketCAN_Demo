// 使用C++ 11的语言编写代码
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <stdexcept>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if.h>
#include <linux/sockios.h>
#include <unistd.h>

// 使用C++的CANdb++库来解析CAN的物理值
// 参考：https://github.com/GENIVI/CANdb
#include "candbcpp/CANdb.h"

// 定义一些常量
#define DEFAULT_CAN_INTERFACE "can0"
#define DEFAULT_SEND_TIMES 100

// 创建CANFrame类，封装了can_frame结构体
class CANFrame
{
public:
    // 构造函数，初始化can_frame结构体
    CANFrame(uint32_t id = 0, uint8_t dlc = 0, const uint8_t *data = nullptr)
    {
        frame_.can_id = id;
        frame_.can_dlc = dlc;
        if (data != nullptr)
        {
            std::copy(data, data + dlc, frame_.data);
        }
    }

    // 拷贝构造函数，复制can_frame结构体
    CANFrame(const CANFrame &other)
    {
        frame_ = other.frame_;
    }

    // 赋值运算符，复制can_frame结构体
    CANFrame &operator=(const CANFrame &other)
    {
        if (this != &other)
        {
            frame_ = other.frame_;
        }
        return *this;
    }

    // 获取can_frame结构体的指针，用于传递给socket函数
    struct can_frame *get_ptr()
    {
        return &frame_;
    }

    // 获取can_id
    uint32_t get_id() const
    {
        return frame_.can_id;
    }

    // 获取can_dlc
    uint8_t get_dlc() const
    {
        return frame_.can_dlc;
    }

    // 获取data数组的指针
    const uint8_t *get_data() const
    {
        return frame_.data;
    }

private:
    struct can_frame frame_; // can_frame结构体，存储了CAN帧的信息
};

// 创建CANSocket类，封装了socket相关的操作
class CANSocket
{
public:
    // 构造函数，初始化socket，并绑定到指定的接口
    CANSocket(const std::string &interface) : interface_(interface)
    {
        fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd_ < 0)
        {
            throw std::runtime_error("Try to create can socket failed: " + std::string(strerror(errno)));
        }
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = get_can_if_index(interface_);
        if (bind(fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            throw std::runtime_error("Try to bind can socket failed: " + std::string(strerror(errno)));
        }
    }

    // 析构函数，关闭socket
    ~CANSocket()
    {
        close(fd_);
    }

    // 发送一个CAN帧到指定的接口
    void send(const CANFrame &frame)
    {
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = get_can_if_index(interface_);
        int nbytes = sendto(fd_, frame.get_ptr(), sizeof(struct can_frame), 0, (struct sockaddr *)&addr, sizeof(addr));
        if (nbytes < 0)
        {
            throw std::runtime_error("Try to send can frame failed: " + std::string(strerror(errno)));
        }
    }

    // 接收一个CAN帧，并返回接收到的接口名和CAN帧对象
    std::pair<std::string, CANFrame> receive()
    {
        struct sockaddr_can addr;
        socklen_t len = sizeof(addr);
        struct can_frame frame;
        int nbytes = recvfrom(fd_, &frame, sizeof(struct can_frame), 0, (struct sockaddr *)&addr, &len);
        if (nbytes < 0)
        {
            throw std::runtime_error("Try to receive can frame failed: " + std::string(strerror(errno)));
        }
        // 获取接口名
        struct ifreq ifr;
        ifr.ifr_ifindex = addr.can_ifindex;
        ioctl(fd_, SIOCGIFNAME, &ifr);
        std::string interface(ifr.ifr_name);
        // 创建CANFrame对象
        CANFrame can_frame(frame.can_id, frame.can_dlc, frame.data);
        return std::make_pair(interface, can_frame);
    }

    // 设置过滤器，只接收符合条件的CAN帧
    void set_filter(uint32_t id, uint32_t mask)
    {
        struct can_filter rfilter;
        /* <received_can_id> & mask == can_id & mask */
        rfilter.can_id = id;
        rfilter.can_mask = mask;
        if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0)
        {
            throw std::runtime_error("Try to set can filter failed: " + std::string(strerror(errno)));
        }
    }

private:
    // 获取指定接口的索引
    int get_can_if_index(const std::string &interface)
    {
        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strcpy(ifr.ifr_name, interface.c_str());
        if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0)
        {
            throw std::runtime_error("Try to get can interface index failed: " + std::string(strerror(errno)));
        }
        return ifr.ifr_ifindex;
    }

    int fd_;                // socket文件描述符
    std::string interface_; // 绑定的接口名
};

// 创建一个结构体，存储消息和信号的信息
struct MessageInfo
{
    uint32_t id;                                            // 消息的ID
    uint8_t dlc;                                            // 消息的长度
    std::string name;                                       // 消息的名称
    std::unordered_map<std::string, CANdb::Signal> signals; // 消息包含的信号，以信号名为键，信号对象为值
};

// 创建一个类，用于解析和编码CAN帧中的数据
class CANParser
{
public:
    // 构造函数，加载CANdb文件，并创建消息和信号的哈希表
    CANParser(const std::string &db_file)
    {
        db_ = CANdb::loadDBC(db_file); // 加载CANdb文件，返回一个数据库对象
        for (const auto &message : db_.messages)
        {                             // 遍历数据库中的所有消息
            MessageInfo info;         // 创建一个消息信息结构体
            info.id = message.id;     // 设置消息ID
            info.dlc = message.dlc;   // 设置消息长度
            info.name = message.name; // 设置消息名称
            for (const auto &signal : message.signals)
            {                                       // 遍历消息中的所有信号
                info.signals[signal.name] = signal; // 将信号名和信号对象存入哈希表中
            }
            messages_[message.id] = info; // 将消息ID和消息信息结构体存入哈希表中
        }
    }

    // 解析一个CAN帧中的数据，返回一个包含信号名和物理值的哈希表
    std::unordered_map<std::string, double> parse(const CANFrame &frame)
    {
        std::unordered_map<std::string, double> result; // 创建一个结果哈希表
        uint32_t id = frame.get_id();                   // 获取CAN帧的ID
        uint8_t dlc = frame.get_dlc();                  // 获取CAN帧的长度
        const uint8_t *data = frame.get_data();         // 获取CAN帧的数据指针
        if (messages_.count(id) == 0)
        { // 如果消息ID不存在于哈希表中，说明无法解析
            throw std::invalid_argument("Unknown message id: " + std::to_string(id));
        }
        const MessageInfo &info = messages_[id]; // 获取消息信息结构体的引用
        if (dlc != info.dlc)
        { // 如果CAN帧的长度和消息的长度不匹配，说明无法解析
            throw std::invalid_argument("Mismatched message dlc: " + std::to_string(dlc));
        }
        for (const auto &pair : info.signals)
        {                                              // 遍历消息中的所有信号
            const std::string &name = pair.first;      // 获取信号名
            const CANdb::Signal &signal = pair.second; // 获取信号对象的引用
            uint64_t raw_value = 0;                    // 存储信号的原始值
            if (signal.byte_order == CANdb::ByteOrder::Motorola)
            { // 如果信号是大端模式（Motorola）
                for (int i = 0; i < signal.bit_size; i++)
                {                                                          // 遍历信号占用的位数
                    int bit_pos = signal.start_bit + i;                    // 计算信号在CAN帧中的位位置
                    int byte_pos = bit_pos / 8;                            // 计算信号在CAN帧中的字节位置
                    int bit_offset = bit_pos % 8;                          // 计算信号在字节中的位偏移量
                    raw_value <<= 1;                                       // 将原始值左移一位，为下一位腾出空间
                    raw_value |= (data[byte_pos] >> (7 - bit_offset)) & 1; // 将信号对应的位的值加入到原始值中
                }
            }
            else
            { // 如果信号是小端模式（Intel）
                for (int i = signal.bit_size - 1; i >= 0; i--)
                {                                                    // 遍历信号占用的位数，从高位到低位
                    int bit_pos = signal.start_bit + i;              // 计算信号在CAN帧中的位位置
                    int byte_pos = bit_pos / 8;                      // 计算信号在CAN帧中的字节位置
                    int bit_offset = bit_pos % 8;                    // 计算信号在字节中的位偏移量
                    raw_value <<= 1;                                 // 将原始值左移一位，为下一位腾出空间
                    raw_value |= (data[byte_pos] >> bit_offset) & 1; // 将信号对应的位的值加入到原始值中
                }
            }
            double physical_value = signal.offset + signal.factor * raw_value; // 根据公式计算信号的物理值
            result[name] = physical_value;                                     // 将信号名和物理值存入结果哈希表中
        }
        return result;
    }

    // 编码一个包含信号名和物理值的哈希表，返回一个CAN帧对象
    CANFrame encode(uint32_t id, const std::unordered_map<std::string, double> &values)
    {
        if (messages_.count(id) == 0)
        { // 如果消息ID不存在于哈希表中，说明无法编码
            throw std::invalid_argument("Unknown message id: " + std::to_string(id));
        }
        const MessageInfo &info = messages_[id]; // 获取消息信息结构体的引用
        uint8_t dlc = info.dlc;                  // 获取消息的长度
        uint8_t data[8] = {0};                   // 创建一个数据数组，初始化为全0
        for (const auto &pair : values)
        {                                         // 遍历输入的哈希表中的所有键值对
            const std::string &name = pair.first; // 获取信号名
            double physical_value = pair.second;  // 获取物理值
            if (info.signals.count(name) == 0)
            { // 如果信号名不存在于消息中，说明无法编码
                throw std::invalid_argument("Unknown signal name: " + name);
            }
            const CANdb::Signal &signal = info.signals.at(name);                   // 获取信号对象的引用
            uint64_t raw_value = (physical_value - signal.offset) / signal.factor; // 根据公式计算信号的原始值
            if (signal.byte_order == CANdb::ByteOrder::Motorola)
            { // 如果信号是大端模式（Motorola）
                for (int i = 0; i < signal.bit_size; i++)
                {                                                                                         // 遍历信号占用的位数
                    int bit_pos = signal.start_bit + i;                                                   // 计算信号在CAN帧中的位位置
                    int byte_pos = bit_pos / 8;                                                           // 计算信号在CAN帧中的字节位置
                    int bit_offset = bit_pos % 8;                                                         // 计算信号在字节中的位偏移量
                    data[byte_pos] |= ((raw_value >> (signal.bit_size - 1 - i)) & 1) << (7 - bit_offset); // 将信号对应的位的值加入到数据数组中
                }
            }
            else
            { // 如果信号是小端模式（Intel）
                for (int i = signal.bit_size - 1; i >= 0; i--)
                {                                                                                   // 遍历信号占用的位数，从高位到低位
                    int bit_pos = signal.start_bit + i;                                             // 计算信号在CAN帧中的位位置
                    int byte_pos = bit_pos / 8;                                                     // 计算信号在CAN帧中的字节位置
                    int bit_offset = bit_pos % 8;                                                   // 计算信号在字节中的位偏移量
                    data[byte_pos] |= ((raw_value >> (signal.bit_size - 1 - i)) & 1) << bit_offset; // 将信号对应的位的值加入到数据数组中
                }
            }
        }
        CANFrame frame(id, dlc, data); // 创建一个CANFrame对象，传入ID，长度和数据数组
        return frame;
    }

private:
    CANdb::DBC db_;                                      // CANdb数据库对象，存储了CANdb文件中的信息
    std::unordered_map<uint32_t, MessageInfo> messages_; // 消息哈希表，以消息ID为键，消息信息结构体为值
};

// 创建一个类，用于缓存和管理接收和发送的数据
class CANBuffer
{
public:
    // 构造函数，初始化缓冲区大小和socket对象
    CANBuffer(size_t size, std::shared_ptr<CANSocket> socket) : size_(size), socket_(socket)
    {
        buffer_ = new CANFrame[size_]; // 动态分配内存空间给缓冲区数组
        head_ = tail_ = count_ = 0;    // 初始化头尾指针和计数器为0
    }

    // 析构函数，释放缓冲区数组的内存空间
    ~CANBuffer()
    {
        delete[] buffer_;
    }

    // 将一个CAN帧对象加入到缓冲区中
    void push(const CANFrame &frame)
    {
        if (is_full())
        { // 如果缓冲区已满，抛出异常
            throw std::runtime_error("Buffer is full");
        }
        buffer_[tail_] = frame;      // 将CAN帧对象复制到缓冲区的尾部
        tail_ = (tail_ + 1) % size_; // 更新尾指针，如果到达数组末尾，回到数组开头
        count_++;                    // 更新计数器
    }

    // 从缓冲区中取出一个CAN帧对象，并返回
    CANFrame pop()
    {
        if (is_empty())
        { // 如果缓冲区为空，抛出异常
            throw std::runtime_error("Buffer is empty");
        }
        CANFrame frame = buffer_[head_]; // 获取缓冲区头部的CAN帧对象
        head_ = (head_ + 1) % size_;     // 更新头指针，如果到达数组末尾，回到数组开头
        count_--;                        // 更新计数器
        return frame;
    }

    // 判断缓冲区是否为空
    bool is_empty() const
    {
        return count_ == 0;
    }

    // 判断缓冲区是否已满
    bool is_full() const
    {
        return count_ == size_;
    }

    // 获取缓冲区中的元素个数
    size_t get_count() const
    {
        return count_;
    }

    // 获取socket对象的指针
    std::shared_ptr<CANSocket> get_socket() const
    {
        return socket_;
    }

private:
    size_t size_;                       // 缓冲区大小
    CANFrame *buffer_;                  // 缓冲区数组，用于存储CAN帧对象
    size_t head_;                       // 缓冲区头指针，指向第一个元素
    size_t tail_;                       // 缓冲区尾指针，指向最后一个元素的下一个位置
    size_t count_;                      // 缓冲区中的元素个数
    std::shared_ptr<CANSocket> socket_; // socket对象的智能指针，用于接收和发送数据
};

// 定义一个全局变量，存储CANdb文件的路径
const std::string db_file = "example.dbc";

// 定义一个函数，用于发送数据
void send_data()
{
    try
    {
        std::shared_ptr<CANSocket> socket(new CANSocket(DEFAULT_CAN_INTERFACE)); // 创建一个socket对象，并用智能指针管理其生命周期
        CANParser parser(db_file);                                               // 创建一个parser对象，加载CANdb文件
        for (int i = 0; i < DEFAULT_SEND_TIMES; i++)
        {                                                   // 循环发送数据
            std::unordered_map<std::string, double> values; // 创建一个哈希表，存储信号名和物理值
            values["signal1"] = i;                          // 设置信号1的值为i
            values["signal2"] = i * 2;                      // 设置信号2的值为i * 2
            CANFrame frame = parser.encode(0, values);      // 调用parser的encode方法，将哈希表编码为CAN帧对象
            socket->send(frame);                            // 调用socket的send方法，将CAN帧对象发送出去
            usleep(1000);                                   // 等待1毫秒
        }
    }
    catch (const std::exception &e)
    { // 捕获任何异常，并打印错误信息
        std::cerr << "Error in send_data: " << e.what() << std::endl;
    }
}

// 定义一个函数，用于接收数据
void receive_data()
{
    try
    {
        std::shared_ptr<CANSocket> socket(new CANSocket(DEFAULT_CAN_INTERFACE)); // 创建一个socket对象，并用智能指针管理其生命周期
        CANParser parser(db_file);                                               // 创建一个parser对象，加载CANdb文件
        socket->set_filter(1, 1);                                                // 设置过滤器，只接收ID为1的消息
        while (true)
        {                                                                                                              // 循环接收数据
            std::pair<std::string, CANFrame> pair = socket->receive();                                                 // 调用socket的receive方法，返回一个包含接口名和CAN帧对象的pair
            std::string interface = pair.first;                                                                        // 获取接口名
            CANFrame frame = pair.second;                                                                              // 获取CAN帧对象
            std::unordered_map<std::string, double> values = parser.parse(frame);                                      // 调用parser的parse方法，将CAN帧对象解析为一个包含信号名和物理值的哈希表
            std::cout << "Received a CAN frame from interface " << interface << " id " << frame.get_id() << std::endl; // 打印接收到的信息
            std::cout << "SIZE " << frame.get_dlc() << "Bytes, DATA ";                                                 // 打印数据长度和数据内容
            for (int i = 0; i < frame.get_dlc(); i++)
            {
                std::cout << std::hex << (int)frame.get_data()[i] << " ";
            }
            std::cout << std::endl;
            std::cout << "SIGNALS "; // 打印信号名和物理值
            for (const auto &pair : values)
            {
                std::cout << pair.first << ": " << pair.second << ", ";
            }
            std::cout << std::endl;
        }
    }
    catch (const std::exception &e)
    { // 捕获任何异常，并打印错误信息
        std::cerr << "Error in receive_data: " << e.what() << std::endl;
    }
}

int main(int argc, char **argv)
{
    system("ip link set " DEFAULT_CAN_INTERFACE " type can bitrate 1000000"); // 设置CAN接口的类型和波特率
    system("ip link set " DEFAULT_CAN_INTERFACE " up");                       // 启动CAN接口
#ifdef CANSEND
    send_data(); // 如果定义了CANSEND宏，调用发送数据的函数
#else
    receive_data(); // 否则，调用接收数据的函数
#endif
    return 0;
}
