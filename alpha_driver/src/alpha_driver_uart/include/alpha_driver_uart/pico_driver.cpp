/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Authors: 
      Lin Zhao <linzhao@uri.edu>
    Year: 2023-2023

    Copyright (C) 2023-2023 Smart Ocean Systems Laboratory
*/

#include <alpha_driver_uart/pico_driver.h> 

PicoDriver::PicoDriver(): close_(false) {
    try
    {
        // make serial object
        serial_ = std::make_shared<serial::Serial>(
            DEFAULT_PORT, DEFAULT_BAUD, serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT)); 

        // flush the IO buffer
        serial_->flush();
    }
    catch (serial::IOException& e)
    {
        printf("Pico Driver: serial issue !\n");
        std::exit(EXIT_FAILURE);
    }  

    // setup the receive thread
    std::thread t(std::bind(&PicoDriver::ReceiveLoop, this));
    t.detach();
}

PicoDriver::PicoDriver(const SerialParam &param): close_(false) {
    try
    {
        serial_ = std::make_shared<serial::Serial>(); 

        serial::Timeout to = serial::Timeout::simpleTimeout(param.timeout);
        serial_->setPort(param.port);
        serial_->setBaudrate(param.baud);
        serial_->setTimeout(to);
        serial_->setBytesize((serial::bytesize_t)param.databits);
        serial_->setParity((serial::parity_t)param.parity);
        serial_->setStopbits((serial::stopbits_t)param.stopbits);
        serial_->setFlowcontrol(serial::flowcontrol_none);
        serial_->open();

        // flush the IO buffer
        serial_->flush();      
    }
    catch (serial::IOException& e)
    {
        std::cerr << "Pico Driver: serial error = " << e.what() <<std::endl;
        std::exit(EXIT_FAILURE);
    }  

    if(!serial_->isOpen()){
        std::cout<< "Pico Driver: port not open\n";
        std::exit(EXIT_FAILURE);
    }

    // setup the receive thread
    std::thread t(std::bind(&PicoDriver::ReceiveLoop, this));
    t.detach(); 
}

PicoDriver::~PicoDriver() {

    if(serial_->isOpen()) {
        close_ = true;
        serial_->close();
    }
}

void PicoDriver::ReceiveLoop() {
    std::string eol;
    eol.append(1,0xd); // '\r'
    eol.append(1,0xa); // '\n'

    while(!close_) {
        // receive
        if(serial_->available()){
            // read one line
            auto line = serial_->readline(65536, eol);

            // send to callback function
            if(serial_callback_) {
              serial_callback_(line);
            }
        }

        // //sleep 1 millisecond, really need that to save some cost ???
        // std::chrono::milliseconds dura(1);
        // std::this_thread::sleep_for(dura);
    }
}

size_t PicoDriver::SendLine(const std::string &str) {
    auto str_send  = str + "\r\n";

    return serial_->write(str_send);
}
