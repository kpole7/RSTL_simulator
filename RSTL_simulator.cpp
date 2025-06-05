#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <ctype.h>
#include <chrono>
#include <sys/select.h>
#include <inttypes.h>
#include <assert.h>

#define RSTL_HARDWARE_SPEED			B600
#define TEXT_HARDWARE_SPEED			"600 no flow control"

#define FRAME_LENGTH				4
#define FRAMES_NUMBER				3

const uint8_t FrameToBeSent1[FRAME_LENGTH] = { '?', 'M', 13, 10 }; // inquiry of software revision, model, serial number
const uint8_t FrameToBeSent2[FRAME_LENGTH] = { '?', 'O', 13, 10 }; // inquiry of local or remote operation
const uint8_t FrameToBeSent3[FRAME_LENGTH] = { '?', 'V', 13, 10 };

const uint8_t* FramesPtr[FRAMES_NUMBER] = { FrameToBeSent1, FrameToBeSent2, FrameToBeSent3 };

static int configureSerialPort(const char *DeviceName);

int SerialDevice;

int main() {
	SerialDevice = configureSerialPort( "/dev/ttyS4" );
	if (SerialDevice < 0){
		std::cout << "Port not opened" << std::endl;
		return 0;
	}
    std::cout << "Port open and configured; handler=" << SerialDevice
    		<< "  baudrate=" << TEXT_HARDWARE_SPEED << std::endl;

    struct termios oldSettings, newSettings;
    tcgetattr(STDIN_FILENO, &oldSettings);
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON | ECHO); // non-canonical mode
    tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);

	auto TimeStart = std::chrono::high_resolution_clock::now();
    auto TimeNow = TimeStart;

    uint32_t TestCounter = 0;

    while (true) {
        fd_set KeyboardReadFds;
        FD_ZERO(&KeyboardReadFds);
        FD_SET(STDIN_FILENO, &KeyboardReadFds);

        struct timeval KeyboardTimeout = {0, 0}; // wait 0 seconds
        int KeyboardState = select(STDIN_FILENO + 1, &KeyboardReadFds, nullptr, nullptr, &KeyboardTimeout);
        if (KeyboardState > 0 && FD_ISSET(STDIN_FILENO, &KeyboardReadFds)) {
            char KeyCode;
            if (read(STDIN_FILENO, &KeyCode, 1) == 1) {
                std::cout << "Pressed: " << (int)KeyCode;
                if (0 != isprint(KeyCode)){
                	std::cout << " " << KeyCode;
                }
                std::cout << "\n";
        		if ((27 == KeyCode) || ('q' == KeyCode) || (1 == KeyCode)){  // Esc, q, Ctrl+A
                    break;
                }
        		if (('1' <= KeyCode) && (KeyCode < '1'+FRAMES_NUMBER)){
        			int Index = KeyCode - '1';
        			assert( Index < FRAMES_NUMBER );
        			assert( FRAMES_NUMBER == (int)(sizeof(FramesPtr)/sizeof(FramesPtr[0])));

        			int n = write(SerialDevice, FramesPtr[Index], FRAME_LENGTH);
    				TimeStart = std::chrono::high_resolution_clock::now();
    				TestCounter = 0;
        			if (FRAME_LENGTH == n){
        				std::cout << "Frame sent successfully";
        			}
        			else{
                    	std::cout << "Error sending frame";
        			}
    				std::cout << " (";
    				for (int J=0; J < FRAME_LENGTH; J++){
    					std::cout << (char)(((FramesPtr[Index][J] >= ' ') && (FramesPtr[Index][J] <= 'z'))? FramesPtr[Index][J] : '.');
    				}
    				std::cout << ")" << std::endl;
        		}
            }
        }

        fd_set SerialReadFds;
		FD_ZERO(&SerialReadFds);
		FD_SET(SerialDevice, &SerialReadFds);

		struct timeval SerialTimeout = { 0, 0 };

		int SerialPortState = select(SerialDevice + 1, &SerialReadFds, nullptr, nullptr, &SerialTimeout);

		if (SerialPortState == -1) {
			std::cerr << "Error: select()" << std::endl;
			break;
		}
		else if (SerialPortState > 0 && FD_ISSET(SerialDevice, &SerialReadFds)) {
			char ReceivedBytes[30];
			int NumberOfReceived = read(SerialDevice, &ReceivedBytes[0], sizeof(ReceivedBytes));

			if (NumberOfReceived > 0) {
				assert( NumberOfReceived <= sizeof(ReceivedBytes));
				TimeNow = std::chrono::high_resolution_clock::now();
				auto Duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeNow - TimeStart);
				std::cout << "Time " << std::dec << Duration.count() << "ms " << (int)TestCounter << "  Received " << NumberOfReceived << " byte(s)   ";
				for (int J=0; J < NumberOfReceived; J++){
					std::cout << std::hex << ((unsigned)ReceivedBytes[J]) % 256u << " ";
				}
				std::cout << " (";
				for (int J=0; J < NumberOfReceived; J++){
					std::cout << (((ReceivedBytes[J] >= ' ') && (ReceivedBytes[J] <= 'z'))? ReceivedBytes[J] : '.');
				}
				std::cout << ")" << std::endl;
			}
			else if (NumberOfReceived == -1) {
				std::cerr << "Error: read()" << std::endl;
			}
			else{
				std::cerr << "Error: received bytes " << NumberOfReceived << std::endl;
			}
		}
		else{
			// There is nothing to do
		}

        usleep(100);
        TestCounter++;
    }

	close( SerialDevice );
    std::cout << "Port closed" << std::endl;

    tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    return 0;
}

#define MY_FLOW_CONTROL	1

// This function opens and configures a serial port
static int configureSerialPort(const char *DeviceName){
    int FileHandler;
    struct termios PortSettings;

    FileHandler = open(DeviceName, O_RDWR | O_NOCTTY | O_SYNC  | O_NONBLOCK);
    if (FileHandler == -1) {
        return -1;
    }

    if (tcgetattr(FileHandler, &PortSettings) != 0) {
        close(FileHandler);
        return -1;
    }

    cfsetispeed(&PortSettings, RSTL_HARDWARE_SPEED);
    cfsetospeed(&PortSettings, RSTL_HARDWARE_SPEED);

    PortSettings.c_oflag = 0;                            // Disable output processing
    PortSettings.c_lflag = 0;                            // Mode 'raw'

#if MY_FLOW_CONTROL == 1
    // no flow control
    PortSettings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);

    PortSettings.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD | CRTSCTS);
    PortSettings.c_cflag |= (CS8 | CREAD | CLOCAL);
#endif

#if MY_FLOW_CONTROL == 2
    // hardware flow control
    PortSettings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);

    PortSettings.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD );
    PortSettings.c_cflag |= (CS8 | CREAD | CLOCAL | CRTSCTS);
#endif

#if MY_FLOW_CONTROL == 3
    // software flow control
    PortSettings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    PortSettings.c_iflag |= ~(IXON | IXOFF | IXANY);

    PortSettings.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD | CRTSCTS);
    PortSettings.c_cflag |= (CS8 | CREAD | CLOCAL);
#endif

    PortSettings.c_cc[VMIN]  = 0;                        // Minimum number of bytes to read
    PortSettings.c_cc[VTIME] = 0;                        // Timeout in tenth of a second

    // apply the configuration
    if (tcsetattr(FileHandler, TCSANOW, &PortSettings) != 0) {
        close(FileHandler);
        return -1;
    }

    return FileHandler;
}


