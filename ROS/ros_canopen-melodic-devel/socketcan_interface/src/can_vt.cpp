#include <socketcan_interface/vt.h>
#include <socketcan_interface/string.h>

using namespace can;

#include <iostream>

/* Frame.msg:
	Header header     0
	uint32 id         0 
	bool is_rtr       0
	bool is_extended  0
	bool is_error     0
	uint8 dlc         3
	uint8[8] data     [01,02,03]
*/

/*
	argv[0] : socketcan_vt 
	argv[1] : device no can0/1 ?
	argv[2] : period
	argv[3] : header : frame id ?
        argv[4] : data
*/

int main(int argc, char *argv[]){
    VTsocket vt; // viewtool

    int extra_frames = argc - 4; 
    std::cout << "main: can vt extra_frames:" << extra_frames << std::endl;
    if(extra_frames < 0){
        std::cout << "usage: "<< argv[0] << " DEVICE PERIOD HEADER#DATA [DATA*]" << std::endl;
        return 1;
    }

    if(!vt.init(argv[1])){
        std::cout << "main: can vt init: failed : " << argv[1] << std::endl;
        return 2;
    }

    int num_frames = 1; // extra_frames+1;
    Frame *frames = new Frame[num_frames];
    std::cout << "argv[3]" <<argv[3]<< std::endl;
    
/*    frames[0].id = 0x155;
    frames[0].is_rtr = 0;
    frames[0].is_extended = 0;
    frames[0].is_error = 0;
    frames[0].dlc = 8;
    for(int i=0; i < frames[0].dlc; i++){
    	frames[0].data[i] = i*2;
    }*/

    for(int i = 0; i < num_frames; ++i){
        std::cout << frames[i] << std::endl;
    }
    
    if(vt.startTX(boost::chrono::duration<double>(atof(argv[2])), num_frames, frames))
    {
        std::cout << "main: can vt startTX: failed 0" << std::endl;
        pause();
        std::cout << "main: can vt startTX: failed af pause" << std::endl;
        return 0;
    }
    
    std::cout << "main: can vt startTX return: 4" << std::endl;
    return 4;
}
