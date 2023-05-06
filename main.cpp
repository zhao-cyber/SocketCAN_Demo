#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if.h>
#include <linux/sockios.h>
#include <string.h>
#include <unistd.h>
#include <unordered_map> // for hash table
#include <queue>         // for buffer
#include <memory>        // for smart pointer
#include "candb++.h"     // for CANdb++ library

#define DEFAULT_CAN_INTERFACE "can0"
#define DEFAULT_SEND_TIMES 100

// a class to represent a CAN frame
class CANFrame
{
public:
    // constructor with parameters
    CANFrame(uint32_t id, uint8_t dlc, const uint8_t *data)
    {
        frame_.can_id = id;
        frame_.can_dlc = dlc;
        memcpy(frame_.data, data, dlc);
    }

    // constructor with struct can_frame
    CANFrame(const struct can_frame &frame)
    {
        frame_ = frame;
    }

    // copy constructor
    CANFrame(const CANFrame &other)
    {
        frame_ = other.frame_;
    }

    // move constructor
    CANFrame(CANFrame &&other)
    {
        frame_ = other.frame_;
        other.frame_ = {0};
    }

    // copy assignment operator
    CANFrame &operator=(const CANFrame &other)
    {
        if (this != &other)
        {
            frame_ = other.frame_;
        }
        return *this;
    }

    // move assignment operator
    CANFrame &operator=(CANFrame &&other)
    {
        if (this != &other)
        {
            frame_ = other.frame_;
            other.frame_ = {0};
        }
        return *this;
    }

    // destructor
    ~CANFrame() {}

    // get the id of the frame
    uint32_t get_id() const
    {
        return frame_.can_id;
    }

    // get the data length code of the frame
    uint8_t get_dlc() const
    {
        return frame_.can_dlc;
    }

    // get the data of the frame as a pointer
    const uint8_t *get_data() const
    {
        return frame_.data;
    }

    // get the data of the frame as an integer (little endian)
    uint64_t get_data_int() const
    {
        uint64_t data_int = 0;
        for (int i = 0; i < frame_.can_dlc; i++)
        {
            data_int |= static_cast<uint64_t>(frame_.data[i]) << (i * 8);
        }
        return data_int;
    }

    // set the id of the frame
    void set_id(uint32_t id)
    {
        frame_.can_id = id;
    }

    // set the data length code of the frame
    void set_dlc(uint8_t dlc)
    {
        frame_.can_dlc = dlc;
    }

    // set the data of the frame from an integer (little endian)
    void set_data_int(uint64_t data_int)
    {
        for (int i = 0; i < frame_.can_dlc; i++)
        {
            frame_.data[i] = static_cast<uint8_t>(data_int >> (i * 8));
        }
    }

    // get the struct can_frame
    const struct can_frame &get_frame() const
    {
        return frame_;
    }

private:
    struct can_frame frame_; // the underlying can frame
};

// a class to represent a CAN socket
class CANSocket
{
public:
    // constructor with parameters
    CANSocket(const char *interface)
    {
        fd_ = socket_init(interface);
        if (fd_ < 0)
        {
            throw std::runtime_error("socket init failed");
        }
        interface_ = interface;
    }

    // destructor
    ~CANSocket()
    {
        socket_release(fd_);
    }

    // send a CAN frame
    int send_frame(const CANFrame &frame)
    {
        return can_frame_send(fd_, interface_, &frame.get_frame());
    }

    // receive a CAN frame
    CANFrame receive_frame()
    {
        struct can_frame frame;
        int nbytes = recvfrom(fd_, &frame, sizeof(frame), 0, nullptr, nullptr);
        if (nbytes < 0)
        {
            throw std::runtime_error("read failed");
        }
        return CANFrame(frame);
    }

    // set filter for a specific id
    void set_filter(uint32_t id, uint32_t mask)
    {
        can_filter_set(fd_, id, mask);
    }

private:
    int fd_;                // the file descriptor of the socket
    const char *interface_; // the name of the interface

    // initialize the socket
    int socket_init(const char *interface)
    {
        int can_fd;
        int ret;
        int if_index = 0;
        struct sockaddr_can addr;
        can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_fd < 0)
        {
            fprintf(stderr, "Try to create can socket failed, %s...\n", strerror(errno));
            return -1;
        }
        if (NULL != interface)
            if_index = get_can_if_index(can_fd, interface);
        addr.can_family = AF_CAN;
        /* index 0 means to bind to all interfaces */
        addr.can_ifindex = if_index;
        bind(can_fd, (struct sockaddr *)&addr, sizeof(addr));
        return can_fd;
    }

    // release the socket
    void socket_release(int fd)
    {
        close(fd);
    }

    // send a can frame with struct can_frame
    int can_frame_send(int fd, const char *interface, const struct can_frame *frame)
    {
        struct sockaddr_can addr;
        socklen_t len = sizeof(addr);
        int nbytes;
        /* assigned interface name */
        if (!interface)
            return -1;
        addr.can_ifindex = get_can_if_index(fd, interface);
        addr.can_family = AF_CAN;
        nbytes = sendto(fd, frame, sizeof(struct can_frame), 0, (struct sockaddr *)&addr, sizeof(addr));
        return nbytes;
    }

    // get the index of the can interface
    int get_can_if_index(int fd, const char *interface)
    {
        int ret;
        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strcpy(ifr.ifr_name, interface);
        ioctl(fd, SIOCGIFINDEX, &ifr);
        ret = ifr.ifr_ifindex;
        return ret;
    }

    // set filter for a specific id
    void can_filter_set(int fd, uint32_t id, uint32_t mask)
    {
        struct can_filter rfilter;
        /* <received_can_id> & mask == can_id & mask */
        rfilter.can_id = id;
        rfilter.can_mask = mask;
        setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
        return;
    }
};

// a class to represent a CAN message with signals
class CANMessage
{
public:
    // constructor with parameters
    CANMessage(uint32_t id, const std::string &name, const std::vector<CANSignal> &signals)
    {
        id_ = id;
        name_ = name;
        signals_ = signals;
    }

    // copy constructor
    CANMessage(const CANMessage &other)
    {
        id_ = other.id_;
        name_ = other.name_;
        signals_ = other.signals_;
    }

    // move constructor
    CANMessage(CANMessage &&other)
    {
        id_ = other.id_;
        name_ = std::move(other.name_);
        signals_ = std::move(other.signals_);
    }

    // copy assignment operator
    CANMessage &operator=(const CANMessage &other)
    {
        if (this != &other)
        {
            id_ = other.id_;
            name_ = other.name_;
            signals_ = other.signals_;
        }
        return *this;
    }

    // move assignment operator
    CANMessage &operator=(CANMessage &&other)
    {
        if (this != &other)
        {
            id_ = other.id_;
            name_ = std::move(other.name_);
            signals_ = std::move(other.signals_);
        }
        return *this;
    }

    // destructor
    ~CANMessage() {}

    // get the id of the message
    uint32_t get_id() const
    {
        return id_;
    }

    // get the name of the message
    const std::string &get_name() const
    {
        return name_;
    }

    // get the signals of the message as a vector
    const std::vector<CANSignal> &get_signals() const
    {
        return signals_;
    }

    // get the signal of the message by name
    const CANSignal &get_signal(const std::string &name) const
    {
        for (const auto &signal : signals_)
        {
            if (signal.get_name() == name)
            {
                return signal;
            }
        }
        throw std::invalid_argument("signal not found");
    }

private:
    uint32_t id_;                    // the id of the message
    std::string name_;               // the name of the message
    std::vector<CANSignal> signals_; // the signals of the message
};

// a class to represent a CAN database with messages and signals
class CANDatabase
{
public:
    // constructor with parameters
    CANDatabase(const std::string &dbc_file)
    {
        db_ = candb_read_file(dbc_file.c_str());
        if (!db_)
        {
            throw std::runtime_error("failed to read dbc file");
        }
        load_messages();
    }

    // destructor
    ~CANDatabase()
    {
        candb_free(db_);
    }

    // get the messages of the database as a vector
    const std::vector<CANMessage> &get_messages() const
    {
        return messages_;
    }

    // get the message of the database by id
    const CANMessage &get_message(uint32_t id) const
    {
        auto it = message_map_.find(id);
        if (it != message_map_.end())
        {
            return messages_[it->second];
        }
        throw std::invalid_argument("message not found");
    }

private:
    candb_t *db_;                                      // the underlying candb object
    std::vector<CANMessage> messages_;                 // the messages of the database
    std::unordered_map<uint32_t, size_t> message_map_; // a hash table to map id to index

    // load messages from candb object
    void load_messages()
    {
        size_t num_messages = candb_get_message_count(db_);
        for (size_t i = 0; i < num_messages; i++)
        {
            candb_message_t *msg = candb_get_message_by_index(db_, i);
            uint32_t id = candb_message_get_id(msg);
            const char *name = candb_message_get_name(msg);
            size_t num_signals = candb_message_get_signal_count(msg);
            std::vector<CANSignal> signals;
            for (size_t j = 0; j < num_signals; j++)
            {
                const char *sig_name = candb_signal_get_name(sig);
                uint8_t start_bit = candb_signal_get_start_bit(sig);
                uint8_t bit_size = candb_signal_get_bit_size(sig);
                double factor = candb_signal_get_factor(sig);
                double offset = candb_signal_get_offset(sig);
                double min_value = candb_signal_get_min_value(sig);
                double max_value = candb_signal_get_max_value(sig);
                const char *unit = candb_signal_get_unit(sig);
                CANSignal signal(sig_name, start_bit, bit_size, factor, offset, min_value, max_value, unit);
                signals.push_back(signal);
            }
            CANMessage message(id, name, signals);
            messages_.push_back(message);
            message_map_[id] = i;
        }
    }
};

// a class to represent a CAN application with socket and database
class CANApplication
{
public:
    // constructor with parameters
    CANApplication(const std::string &dbc_file, const char *interface)
    {
        socket_ = std::make_unique<CANSocket>(interface);    // use smart pointer to manage socket
        database_ = std::make_unique<CANDatabase>(dbc_file); // use smart pointer to manage database
        buffer_ = std::queue<CANFrame>();                    // use queue to buffer frames
    }

    // destructor
    ~CANApplication() {}

    // send a CAN frame
    void send_frame(const CANFrame &frame)
    {
        socket_->send_frame(frame);
    }

    // receive a CAN frame and buffer it
    void receive_frame()
    {
        CANFrame frame = socket_->receive_frame();
        buffer_.push(frame);
    }

    // process the buffered frames and print the physical values
    void process_frames()
    {
        while (!buffer_.empty())
        {
            CANFrame frame = buffer_.front();
            buffer_.pop();
            uint32_t id = frame.get_id();
            uint64_t data_int = frame.get_data_int();
            try
            {
                const CANMessage &message = database_->get_message(id); // use hash table to find message by id
                printf("ID: %x Name: %s Data: %llx\n", id, message.get_name().c_str(), data_int);
                for (const auto &signal : message.get_signals())
                {
                    double value = signal.decode(data_int); // use bit operation to decode value
                    printf("Signal: %s Value: %f %s\n", signal.get_name().c_str(), value, signal.get_unit().c_str());
                }
            }
            catch (const std::invalid_argument &e)
            {
                printf("Unknown message id: %x\n", id);
            }
        }
    }

private:
    std::unique_ptr<CANSocket> socket_;     // the can socket
    std::unique_ptr<CANDatabase> database_; // the can database
    std::queue<CANFrame> buffer_;           // the buffer for frames
};

// the main function
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        fprintf(stderr, "Usage: %s <dbc file>\n", argv[0]);
        return -1;
    }
    std::string dbc_file = argv[1];
    system("ip link set " DEFAULT_CAN_INTERFACE " type can bitrate 1000000");
    system("ip link set " DEFAULT_CAN_INTERFACE " up");
#ifdef CANSEND
    CANApplication app(dbc_file, NULL); // send mode does not need interface name
#else
    CANApplication app(dbc_file, DEFAULT_CAN_INTERFACE); // receive mode needs interface name
#endif

#ifdef CANSEND
    uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    for (int i = 0; i < DEFAULT_SEND_TIMES; i++)
    {
        data[7] = i;
        CANFrame frame(0x123, 8, data); // create a can frame with id 0x123 and data 0x11 0x22 ... 0x88
        app.send_frame(frame);          // send the frame
        sleep(1);                       // send period 1s
    }
#else
    /* only receive can_id == 0x123 frame */
    app.set_filter(0x123, CAN_SFF_MASK); // set filter for id 0x123
    while (1)
    {
        app.receive_frame();  // receive a frame and buffer it
        app.process_frames(); // process the buffered frames and print the physical values
    }
#endif
    return 0;
}