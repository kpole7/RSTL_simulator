#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/select.h>
#include <inttypes.h>

#define RSTL_HARDWARE_SPEED			B19200


const uint8_t FrameToBeSent[] = { 0x01, 0x03, 0x03, 0xE8, 0x00, 0x07, 0x84, 0x78 };

static int configureSerialPort(const char *DeviceName);

int SerialDevice;


int main() {
	SerialDevice = configureSerialPort( "/dev/ttyUSB0" );
	if (SerialDevice < 0){
		std::cout << "Port not opened" << std::endl;
		return 0;
	}
    std::cout << "Port open and configured; handler " << SerialDevice << std::endl;

    struct termios oldSettings, newSettings;
    tcgetattr(STDIN_FILENO, &oldSettings);
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON | ECHO); // Tryb niekanoniczny, bez echa
    tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);

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
        		if ('S' == KeyCode){
        			int n = write(SerialDevice, &FrameToBeSent, sizeof(FrameToBeSent));
        			if (sizeof(FrameToBeSent) == n){
                    	std::cout << "Frame sent successfully" << std::endl;
        			}
        			else{
                    	std::cout << "Error sending frame" << std::endl;
        			}
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
			char ReceivedByte;
			int NumberOfReceived = read(SerialDevice, &ReceivedByte, 1); // Odczytaj 1 bajt

			if (NumberOfReceived == 1) {
				std::cout << "Received: 0x" << std::hex << ((unsigned)ReceivedByte) % 256u << " ('"
						<< (isprint(ReceivedByte) ? ReceivedByte : '.') << "')" << std::endl;
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





        usleep(1000);
    }

	close( SerialDevice );
    std::cout << "Port closed" << std::endl;

    tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    return 0;
}


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

    PortSettings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    PortSettings.c_oflag = 0;                            // Disable output processing
    PortSettings.c_lflag = 0;                            // Mode 'raw'

    PortSettings.c_cflag &= ~(CSIZE | CSTOPB | PARODD | CRTSCTS);
    PortSettings.c_cflag |= (CS8 | PARENB | CREAD | CLOCAL);

    PortSettings.c_cc[VMIN]  = 0;                        // Minimum number of bytes to read
    PortSettings.c_cc[VTIME] = 0;                        // Timeout in tenth of a second

    // apply the configuration
    if (tcsetattr(FileHandler, TCSANOW, &PortSettings) != 0) {
        close(FileHandler);
        return -1;
    }

    return FileHandler;
}


