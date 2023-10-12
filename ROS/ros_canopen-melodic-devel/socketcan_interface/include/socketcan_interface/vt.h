#ifndef H_CAN_VT
#define H_CAN_VT

#include <socketcan_interface/interface.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
// #include <linux/can/bcm.h>
#include <linux/can/error.h>

#include <cstring>

#include <boost/chrono.hpp>
#include <iostream>

namespace can {
#if 0
/*
/**
 * struct bcm_msg_head - head of messages to/from the broadcast manager
 * @opcode:    opcode, see enum below.
 * @flags:     special flags, see below.
 * @count:     number of frames to send before changing interval.
 * @ival1:     interval for the first @count frames.
 * @ival2:     interval for the following frames.
 * @can_id:    CAN ID of frames to be sent or received.
 * @nframes:   number of frames appended to the message head.
 * @frames:    array of CAN frames.
 */
struct bcm_msg_head {
	__u32 opcode;
	__u32 flags;
	__u32 count;
	struct bcm_timeval ival1, ival2;
	canid_t can_id;
	__u32 nframes;
	struct can_frame frames[0];
};
*/
#endif

struct vt_msg_head {
	canid_t can_id;
	// __u32 nframes;
	struct can_frame frames[0];
};

class VTsocket{
    int s_;
    struct Message {
        size_t size;
        uint8_t *data;
        struct can_frame frame;
#if 0
        Message(size_t n)
        : size(sizeof(bcm_msg_head) + sizeof(can_frame)*n), data(new uint8_t[size])
        {
            assert(n<=256);
            std::memset(data, 0, size);
            head().nframes = n;
        }
        bcm_msg_head& head() {
            return *(bcm_msg_head*)data;
        }
        template<typename T> void setIVal2(T period){
            long long usec = boost::chrono::duration_cast<boost::chrono::microseconds>(period).count();
            head().ival2.tv_sec = usec / 1000000;
            head().ival2.tv_usec = usec % 1000000;
        }
   
        void setHeader(Header header){
            head().can_id = header.id | (header.is_extended?CAN_EFF_FLAG:0);
        }

	vt_msg_head& head() {
            return *(vt_msg_head*)data;
        }
 #else       
	Message(size_t n)
        : size( sizeof(can_frame)*n), data(new uint8_t[size])
        {
           assert(n<=256);
            std::memset(data, 0, size);
        }        
#endif     

        bool write(int s){
#if 0            
            return ::write(s, data, size) > 0;
#else
            return ::write(s, &frame, sizeof(frame)) > 0;
/*
	    struct can_frame frame1 = {0};
	    frame1.can_id = 0x101; // id;
	    frame1.can_dlc = 8; // dlc;
	    for (int i = 0; i < frame1.can_dlc; i++)
	    {
	      frame1.data[i]= i;// data[i];
	    }
    
            return ::write(s, &frame1, sizeof(frame1)) > 0;
*/
#endif
        }
        ~Message(){
            delete[] data;
            data = 0;
            size = 0;
        }
    };
public:
    VTsocket():s_(-1){
    }
    bool init(const std::string &device){
        s_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        // s_ = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
        if(s_ < 0 ) {
               std::cout << "s_ = socket <0 \n"<< std::endl;
        	return false;
        }
        
        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, device.c_str());
        std::cout << "device.c_str() : "<<device.c_str()<< std::endl;
        // if (ioctl(s, SIOCGIFMTU, &ifr) < 0) 
        // int ret = ioctl(s_, SIOCGIFMTU, &ifr);
        int ret = ioctl(s_, SIOCGIFINDEX, &ifr);        
        if(ret != 0){
            std::cout << "ret = ioctl !=0 \n"<< std::endl;
            shutdown();
            return false;
        }
#if 1
        struct sockaddr_can addr = {0};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        std::cout << "caddr.can_ifindex : "<< addr.can_ifindex << std::endl;
        //setsockopt(s_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
        ret = bind(s_, (struct sockaddr *)&addr, sizeof(addr));
        //ret = connect(s_, (struct sockaddr *)&addr, sizeof(addr));
        if(ret < 0){
            std::cout << "connect/bind ret : "<< ret << std::endl;
            shutdown();
            return false;
        }
#else
        
#endif
        
        return true;
    }
    template<typename DurationType> bool startTX(DurationType period, /* Header header,*/ size_t num, Frame *frames) {
        Message msg(num);
#if 0
        msg.setHeader(header);        
        msg.setIVal2(period);

        bcm_msg_head &head = msg.head();

        head.opcode = TX_SETUP;
        head.flags |= SETTIMER | STARTTIMER;

        for(size_t i=0; i < num; ++i){ // msg nr
            head.frames[i].can_dlc = frames[i].dlc;
            head.frames[i].can_id = head.can_id;
            for(size_t j = 0; j < head.frames[i].can_dlc; ++j){ // byte nr
                head.frames[i].data[j] = frames[i].data[j];
            }
        }
#else
/*
	vt_msg_head &head = msg.head();
	
        for(size_t i=0; i < num; ++i)
        { // msg nr
            head.frames[i].can_dlc = frames[i].dlc;
            head.frames[i].can_id = 0x101;// frames[i].can_id;//head.can_id;
            for(size_t j = 0; j < head.frames[i].can_dlc; ++j){ // byte nr
                head.frames[i].data[j] = frames[i].data[j];
            }
        }
*/
      	msg.frame={0};          
        msg.frame.can_id = frames[0].id;
        msg.frame.can_dlc = frames[0].dlc;
        for(size_t i=0; i < num; ++i){
	    for(size_t j = 0; j < frames[i].dlc; ++j){ // byte nr
	        msg.frame.data[j] = frames[i].data[j];
	    } 
        }
#endif
        
        std::cout << "startTX : %x : "<< msg.write(s_) << std::endl;
        return msg.write(s_);
    }
    bool stopTX(Header header){   
        Message msg(0);
#if 0         
        msg.head().opcode = TX_DELETE;
        msg.setHeader(header);
#endif
        return msg.write(s_);
    }
    void shutdown(){
        if(s_ > 0){
            close(s_);
            s_ = -1;
        }
    }

    virtual ~VTsocket(){
        shutdown();
    }
};

}

#endif
