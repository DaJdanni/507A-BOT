#pragma once
#include "../vex.h"

/*
session:
    [dataPoint1] = 
    {
        id: string
        data: 
        {
            [time, data]
        }
    }
}
*/
/*
dataCaptures:
    {
        [dataCapture1] = {
            [id]: string
            [key]: data
        }
    }
*/

namespace Bucees 
{

    template<typename K, typename T>
    std::map<K, T> createDataCapture(K key, T value) {
        std::map<K, T> dataCapture;
        dataCapture[key] = value;
        return dataCapture;
    }

    namespace dontTouchThisPleaseIgnoreIt {

        struct dataStructure
        {
            int id;
            std::vector<std::pair<int, double>> data;
        };

        template<typename K, typename T>
        class DataSink
        {

            private:

            virtual double getData(K target, T key) = 0; // temporary function so c++ doesn't cope

            protected:

            template <class F>
            vex::task launch_task(F&& function) {
            return vex::task([](void* parameters) {
                std::unique_ptr<std::function<void()>> ptr{static_cast<std::function<void()>*>(parameters)};
                (*ptr)();
                return 0;
            }, new std::function<void()>(std::forward<F>(function)));
            }

            int timeStep = 500;

            vex::mutex mutex;
            bool recording = false;
            bool mutexLock = false;
            bool sessionStarted = false;
            const std::string headerText = "time, data";

            std::string id;
            std::vector<std::map<K, T>> dataCaptures;

            std::vector<dataStructure> session;

            std::string to_string(int x) {
                std::stringstream ss;
                ss << x;
                return ss.str();
            }

            public:

            DataSink(std::string id, std::vector<std::map<K, T>> dataCaptures, int timeStep = 500) {
                this->id = id;
                this->dataCaptures = dataCaptures;
                this->timeStep = timeStep;
            }

            ~DataSink() {}

            void stop() {
                std::cout << "Ending the recording." << std::endl;
                this->recording = false;
            }

            void start() {

                if (this->sessionStarted == true) {
                    std::cout << "WARNING: USE RESUME INSTEAD TO RESTART THE RECORDING." << std::endl;
                };

                if (this->mutexLock == false) {
                    //std::cout << "locking function" << std::endl;
                    this->mutexLock = true;
                    launch_task([&] {
                        this->start();
                    });
                    wait(10, vex::msec);
                    return;
                }

                this->mutex.lock();
                this->recording = true;
                this->sessionStarted = true;

                std::cout << "Starting to record." << std::endl;
                std::cout << "Size of Data Captures: " << static_cast<int>(this->dataCaptures.size()) << std::endl; // dont removbe this plz idk how it makes code work :(

                for (size_t index = 0; index < this->dataCaptures.size(); ++index) {
                    std::cout << "Creating a data structure." << std::endl;
                    std::vector<std::pair<int, double>> data;
                    dataStructure temp {
                        index,
                        data
                    };
                    this->session.push_back(temp);
                    std::cout << "Pushed back" << std::endl;
                }

                for (int currentTime = 0; currentTime < 9999999999; currentTime = currentTime + this->timeStep) {
                    if (this->recording == false) {
                        //std::cout << "Recording has been requested to stop." << std::endl;
                        break;
                    };
                    
                    for (dataStructure& structure : this->session) {
                        //std::cout << "Recording data for structure: " << structure.id << std::endl;
                        auto dataCapture = this->dataCaptures.at(structure.id);
                        structure.data.push_back(std::make_pair(currentTime, this->getData(dataCapture.begin()->first, dataCapture.begin()->second)));
                        //std::cout << "Data has been recorded. Data Indices: " << structure.data.size() << std::endl;
                    }     
                    wait(this->timeStep, vex::msec);
                }

                return;
            }

            void parseData() {

                std::cout << "Parsing data..." << std::endl;

                vex::brain Brain = vex::brain();

                if (!Brain.SDcard.isInserted()) {
                    std::cout << "WARNING: NO SD CARD IS INSTALLED." << std::endl;
                    return;
                }

                std::cout << "Itterating through every structure... [Amount: " << static_cast<int>(this->session.size()) << "]" << std::endl;         
                for (dataStructure structure : this->session) { // itterate through every structure
                    std::cout << "Data Size: " << static_cast<int>(structure.data.size()) << std::endl;
                    std::string fileName = this->id + "_" + to_string(structure.id) + ".csv"; // create the file name "Data For: X"
                    std::string dataBuffer = this->headerText + " \n"; // create a new data buffer

                    for (const auto& dataPoint : structure.data) {
                        std::cout << "Time: " << dataPoint.first << ", Data: " << dataPoint.second << std::endl;
                        dataBuffer += to_string(dataPoint.first) + ",";
                        dataBuffer += to_string(dataPoint.second) + "\n";
                    }

                    std::vector<uint8_t> byteArray(dataBuffer.begin(), dataBuffer.end());
                    uint8_t* dataBufferConverted = byteArray.data();
                    std::vector<uint8_t> byteArray2(fileName.begin(), fileName.end());
                    uint8_t* fileNameConverted = byteArray2.data();
                    const char* fileNamePtr = fileName.c_str();

                    std::cout << "Saving file to SD Card..." << std::endl;
                    if (Brain.SDcard.savefile(fileName.c_str(), dataBufferConverted, dataBuffer.length()) == 0) {
                        std::cout << "File was not saved to SD Card!" << std::endl;
                    } else {
                        std::cout << "File name: " << fileName << std::endl;
                        //std::cout << "Data Buffer: \n" << dataBuffer << std::endl;  
                        //std::cout << "Data Buffer UINT8_T: \n" << dataBufferConverted << std::endl; 
                        std::cout << "Successfully saved data to SD Card!" << std::endl;
                    }

                    wait(100, vex::msec);
                }
            }
        };
    }

    template<typename K, typename T>
    class DataSink_Motor : public dontTouchThisPleaseIgnoreIt::DataSink<K, T> {
        private:
        double getData(K target, T key) override { // alpha sigma function >:C
            vex::motor* tempMotor = target;
            
            switch (key) {
                case 0: // MOTOR_POSITION_DEG
                    //std::cout << "MOTOR POSITION DEG: " << tempMotor->position(vex::rotationUnits::deg) << std::endl;
                    return tempMotor->position(vex::rotationUnits::deg);
                case 1: // MOTOR_POSITION_REV
                    //std::cout << "MOTOR POSITION REV: " << tempMotor->position(vex::rotationUnits::rev) << std::endl;
                    return tempMotor->position(vex::rotationUnits::rev);
                case 2: // MOTOR_POSITION_RAW
                    //std::cout << "MOTOR POSITION RAW: " << tempMotor->position(vex::rotationUnits::raw) << std::endl;
                    return tempMotor->position(vex::rotationUnits::raw);
                case 3: // MOTOR_VELOCITY_PCT
                    //std::cout << "MOTOR VELOCITY PCT: " << tempMotor->velocity(vex::velocityUnits::pct) << std::endl;
                    return tempMotor->velocity(vex::velocityUnits::pct);
                case 4: // MOTOR_VELOCITY_RPM
                    //std::cout << "MOTOR VELOCITY RPM: " << tempMotor->velocity(vex::velocityUnits::rpm) << std::endl;
                    return tempMotor->velocity(vex::velocityUnits::rpm);
                case 5: // MOTOR_VELOCITY_DPS
                    //std::cout << "MOTOR VELOCITY DPS: " << tempMotor->velocity(vex::velocityUnits::dps) << std::endl;
                    return tempMotor->velocity(vex::velocityUnits::dps);
                case 6: // MOTOR_TORQUE_NM
                    //std::cout << "MOTOR TORQUE NM: " << tempMotor->torque(vex::torqueUnits::Nm) << std::endl;
                    return tempMotor->torque(vex::torqueUnits::Nm);
                case 7: // MOTOR_TEMPERATURE_CEL
                    //std::cout << "MOTOR TEMPERATURE CEL: " << tempMotor->temperature(vex::temperatureUnits::celsius) << std::endl;
                    return tempMotor->temperature(vex::temperatureUnits::celsius);
                case 8: // MOTOR_TEMPERATURE_FAH
                    //std::cout << "MOTOR TEMPERATURE FAH: " << tempMotor->temperature(vex::temperatureUnits::fahrenheit) << std::endl;
                    return tempMotor->temperature(vex::temperatureUnits::fahrenheit);
                default:
                    return -999.999; // u never know maneee
            }
        }

        public:
        DataSink_Motor(std::string id, std::vector<std::map<K, T>> dataCaptures, int timeStep = 500) : dontTouchThisPleaseIgnoreIt::DataSink<K, T>(id, dataCaptures, timeStep) {}
    };

    template<typename K, typename T>
    class DataSink_Robot : public dontTouchThisPleaseIgnoreIt::DataSink<K, T> {
        private:
        double getData(K target, T key) override { // alpha sigma function >:C
            Bucees::Robot* tempRobot = target;
            
            switch (key) {
                case 9: // ODOMETRY_COORDINATE_X
                    //std::cout << "ODOM X COORD: " << tempRobot->getRobotCoordinates().x << std::endl;
                    return static_cast<double>(tempRobot->getRobotCoordinates().x);
                case 10: // ODOMETRY_COORDINATE_Y
                    //std::cout << "ODOM Y COORD: " << tempRobot->getRobotCoordinates().y << std::endl;
                    return static_cast<double>(tempRobot->getRobotCoordinates().y);
                case 11: // ODOMETRY_COORDINATE_ROTATION
                    //std::cout << "ODOM ROTATION (DEGREES): " << tempRobot->getRobotCoordinates(false).theta << std::endl;
                    return static_cast<double>(tempRobot->getRobotCoordinates(false).theta);
                case 12: // ODOMETRY_COORDINATE_X_REVERSED
                    //std::cout << "ODOM X COORD (REVERSED): " << tempRobot->getRobotCoordinates(true, true).x << std::endl;
                    return static_cast<double>(tempRobot->getRobotCoordinates(true, true).x);
                case 13: // ODOMETRY_COORDINATE_Y_REVERSED
                    //std::cout << "ODOM Y COORD (REVERSED): " << tempRobot->getRobotCoordinates(true, true).y << std::endl;
                    return static_cast<double>(tempRobot->getRobotCoordinates(true, true).y);
                case 14: // // ODOMETRY_COORDINATE_ROTATION_RAD
                    //std::cout << "ODOM ROTATION (RADIANS): " << tempRobot->getRobotCoordinates(true, true).theta << std::endl;
                    return static_cast<double>(tempRobot->getRobotCoordinates(true, true).theta);
                default:
                    return -999.999; // u never know maneee
            }
        }

        public:
        DataSink_Robot(std::string id, std::vector<std::map<K, T>> dataCaptures, int timeStep = 500) : dontTouchThisPleaseIgnoreIt::DataSink<K, T>(id, dataCaptures, timeStep) {}
    };
}