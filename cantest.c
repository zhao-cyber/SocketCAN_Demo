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

#define DEFAULT_CAN_INTERFACE "can0"
#define DEFAULT_SEND_TIMES 100

extern int errno;

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

int socket_init(const char *interface)
{
   int can_fd;
   int ret;
   int if_index = 0;
   struct sockaddr_can addr;

   can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
   if (can_fd < 0) {
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

void socket_release(int fd)
{
    close(fd);
}

int can_frame_send(int fd, const char *interface, struct can_frame *frame)
{
    struct sockaddr_can addr;
    socklen_t len = sizeof(addr);
    int nbytes;

    /* assigned interface name */
    if (!interface)
        return -1;

    addr.can_ifindex = get_can_if_index(fd, interface);
    addr.can_family  = AF_CAN;

    nbytes = sendto(fd, frame, sizeof(struct can_frame),
                      0, (struct sockaddr*)&addr, sizeof(addr));

    return nbytes;
}

void can_filter_set(int fd, unsigned int id,unsigned int mask)
{
    struct can_filter rfilter;

    /* <received_can_id> & mask == can_id & mask */
    rfilter.can_id   = id;
    rfilter.can_mask = mask;

    setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    return;
}

void can_recvframe(int fd)
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    socklen_t len = sizeof(addr);
    struct can_frame frame;
    int nbytes;
    int i;
    static unsigned int total_cnt = 0;

    nbytes = recvfrom(fd, &frame, sizeof(struct can_frame),
                      0, (struct sockaddr*)&addr, &len);

    /* get interface name of the received CAN frame */
    ifr.ifr_ifindex = addr.can_ifindex;
    ioctl(fd, SIOCGIFNAME, &ifr);
    printf("Received a CAN frame from interface %s id %d count %u\n", ifr.ifr_name, frame.can_id, total_cnt++);
    printf("SIZE %dBytes, DATA ", frame.can_dlc);
    for (i = 0; i < frame.can_dlc; i++) {
        printf("%#x ", frame.data[i]);
    }
    printf("\n");

    return;
}

int main(int argc, char **argv)
{
    int can_s;
    struct can_frame frame;
    int i;

    system("ip link set " DEFAULT_CAN_INTERFACE " type can bitrate 1000000");
    system("ip link set " DEFAULT_CAN_INTERFACE " up");

#ifdef CANSEND
    can_s = socket_init(NULL);
#else
    can_s = socket_init(DEFAULT_CAN_INTERFACE);
#endif

#ifdef CANSEND
    memset(&frame, 0, sizeof(frame));
    for (i = 0; i < DEFAULT_SEND_TIMES; i++) {
        frame.can_id = 0;
        frame.can_dlc = 8;
        frame.data[7] = i;
        /* send period 1ms */
        usleep(1000);
        can_frame_send(can_s, DEFAULT_CAN_INTERFACE, &frame); 
    }
#else
    /* only receive can_id == 1 frame */
    can_filter_set(can_s, 1, 1);
    while(1) {
        can_recvframe(can_s);
    }
#endif

    socket_release(can_s);

    return 0;
}
