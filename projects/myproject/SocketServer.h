#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sys/socket.h>
#include <thread>
#include <chrono>
#include <iomanip>

#include "PlantArchitecture.h"
#include "RadiationModel.h"
#include "Visualizer.h"

// class TemporaryDirectory
// {
// public:
//     TemporaryDirectory()
//         : m_path()
//     {
//         char dirTemplate[] = "/tmp/helios_temp_dir.XXXXXX";
//         char *temp_dir = mkdtemp(dirTemplate);
//         if (!temp_dir)
//         {
//             perror("Failed to create temporary directory");
//         }
//         else
//         {
//             m_path = temp_dir;
//             std::cout << "Created temporary directory: " << m_path << std::endl;
//         }
//     }

//     ~TemporaryDirectory()
//     {
//         if (!m_path.empty())
//         {
//             std::string cmd = "rm -rf " + m_path;
//             if (system(cmd.c_str()) == -1)
//             {
//                 std::cerr << "Failed to remove temporary directory: " << m_path << std::endl;
//             }
//             else
//             {
//                 std::cout << "Temporary directory cleaned up: " << m_path << std::endl;
//             }
//         }
//     }

//     std::string path() const { return m_path; }

// private:
//     std::string m_path;
// };

class SocketServer
{
public:
    struct Config
    {
        std::string cameraName;
        float cameraDepthMax;
        uint cameraHeight;
        uint cameraWidth;
        float cameraFOV;
    };

    SocketServer(int port, const Config &config, bool useNonBlocking = true)
        : m_port(port),
          m_useNonBlocking(useNonBlocking),
          m_serverFd(-1),
          m_config(config)
    {
    }

    ~SocketServer()
    {
        stop();
    }

    int startAccepting(Visualizer &visualizer, PlantArchitecture &plantarchitecture, RadiationModel *radiation_model = nullptr)
    {
        if (!setupServerSocket())
        {
            return -1;
        }

        while (true)
        {
            bool keepGoing = acceptAndHandleClient(visualizer, plantarchitecture, radiation_model);
            if (!keepGoing)
            {
                break;
            }
        }

        return 0;
    }

    void stop()
    {
        if (m_serverFd >= 0)
        {
            close(m_serverFd);
            m_serverFd = -1;
        }
    }

private:
    bool setupServerSocket()
    {
        // Create socket
        m_serverFd = socket(AF_INET, SOCK_STREAM, 0);
        if (m_serverFd < 0)
        {
            perror("socket() failed");
            return false;
        }

        // Set socket options
        int opt = 1;
        if (setsockopt(m_serverFd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0)
        {
            perror("setsockopt() failed");
            close(m_serverFd);
            m_serverFd = -1;
            return false;
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(m_port);

        // Bind socket
        if (bind(m_serverFd, (struct sockaddr *)(&address), sizeof(address)) < 0)
        {
            perror("bind() failed");
            close(m_serverFd);
            m_serverFd = -1;
            return false;
        }

        // Listen for incoming connections
        if (listen(m_serverFd, 3) < 0)
        {
            perror("listen() failed");
            close(m_serverFd);
            m_serverFd = -1;
            return false;
        }

        std::cout << "Server is listening on Port " << m_port << std::endl;
        return true;
    }

    bool acceptAndHandleClient(Visualizer &visualizer, PlantArchitecture &plantarchitecture, RadiationModel *radiation_model)
    {
        sockaddr_in address{};
        socklen_t addrLen = sizeof(address);

        int clientSocket = accept(m_serverFd, (struct sockaddr *)(&address), &addrLen);
        if (clientSocket < 0)
        {
            perror("accept() failed");
            return true;
        }

        std::cout << "Client connected. Waiting for commands..." << std::endl;

        if (m_useNonBlocking)
        {
            setNonBlocking(clientSocket);
        }

        const size_t BUFFER_SIZE = 4096;
        char buffer[BUFFER_SIZE];
        std::string message;
        message.reserve(BUFFER_SIZE);

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // Timeout of 100 ms

        while (true)
        {
            fd_set readFds;
            FD_ZERO(&readFds);
            FD_SET(clientSocket, &readFds);

            int activity = select(clientSocket + 1, &readFds, NULL, NULL, &timeout);
            if (activity < 0)
            {
                perror("select() error");
                break;
            }

            if (activity == 0 || !FD_ISSET(clientSocket, &readFds))
            {
                continue; // No data, continue waiting
            }

            // Read data from the client
            ssize_t bytesRead = read(clientSocket, buffer, BUFFER_SIZE);
            if (bytesRead < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    continue;
                }
                perror("read() failed");
                std::cout << "Client disconnected. Waiting for new connection..." << std::endl;
                break;
            }
            else if (bytesRead == 0)
            {
                std::cout << "Client disconnected. Waiting for new connection..." << std::endl;
                break;
            }

            message.assign(buffer, bytesRead);

            // Handle the command
            int result = handleCommand(message, clientSocket,
                                       visualizer, plantarchitecture, radiation_model);

            if (result == 1) // Exit command
            {
                close(clientSocket);
                stop();
                return false;
            }
            else if (result < 0) // Error
            {
                break;
            }

            // Clear the buffer
            std::memset(buffer, 0, bytesRead);
        }

        close(clientSocket);
        return true;
    }

    int handleCommand(
        const std::string &command,
        int clientSocket,
        Visualizer &visualizer,
        PlantArchitecture &plantarchitecture,
        RadiationModel *radiation_model)
    {
        if (command == "EXIT")
        {
            std::cout << "Exit command received. Stopping server." << std::endl;
            return 1;
        }

        if (command == "GET_CAMERA_DEPTH_MAX")
        {
            if (!sendFloatValue(clientSocket, m_config.cameraDepthMax))
            {
                perror("Failed to send max depth value");
                return -1;
            }
            return 0;
        }

        if (command == "GET_CAMERA_RESOLUTION")
        {
            std::vector<int> resolution;
            resolution.push_back(m_config.cameraHeight);
            resolution.push_back(m_config.cameraWidth);
            if (!sendInt1DArray(clientSocket, resolution))
            {
                perror("Failed to send camera resolution");
                return -1;
            }
            return 0;
        }

        if (command == "GET_CAMERA_FOV")
        {
            if (!sendFloatValue(clientSocket, m_config.cameraFOV))
            {
                perror("Failed to send camera FOV");
                return -1;
            }
            return 0;
        }

        if (command == "GET_CAMERA_POSE")
        {
            std::vector<float> camera_pose = visualizer.getCameraPose();
            if (!sendFloat1DArray(clientSocket, camera_pose))
            {
                perror("Failed to send camera pose");
                return -1;
            }
            return 0;
        }

        if (command == "GET_RGB")
        {
            std::vector<uint8_t> rgb_data;
            rgb_data = visualizer.getRGBData();

            if (!sendUInt81DArray(clientSocket, rgb_data))
            {
                perror("Failed to send RGB data");
                return -1;
            }
            return 0;
        }

        if (command == "GET_NORM_DEPTH" && radiation_model)
        {
            std::vector<float> depth_data;
            depth_data = radiation_model->getDepthData(m_config.cameraName, m_config.cameraDepthMax, true);

            if (!sendFloat1DArray(clientSocket, depth_data))
            {
                perror("Failed to send normalized depth data");
                return -1;
            }
            return 0;
        }

        if (command == "GET_ACTUAL_DEPTH" && radiation_model)
        {
            std::vector<float> depth_data;
            depth_data = radiation_model->getDepthData(m_config.cameraName, m_config.cameraDepthMax, false);

            if (!sendFloat1DArray(clientSocket, depth_data))
            {
                perror("Failed to send actual depth data");
                return -1;
            }
            return 0;
        }

        if (command == "GET_LABEL_MAP" && radiation_model)
        {
            std::vector<int> label_map;
            label_map = radiation_model->getPrimitiveDataLabelMap(m_config.cameraName, "class", 0);

            if (!sendInt1DArray(clientSocket, label_map))
            {
                perror("Failed to send label map");
                return -1;
            }
            return 0;
        }

        if (command == "GET_BBOX_2D" && radiation_model)
        {
            std::vector<std::vector<float>> bbox_2d;
            bbox_2d = radiation_model->getImageBoundingBoxes_ObjectData(m_config.cameraName, "fruit_id");

            if (!sendFloat2DArray(clientSocket, bbox_2d))
            {
                perror("Failed to send 2D bounding boxes");
                return -1;
            }
            return 0;
        }

        if (command == "SET_CAMERA")
        {
            float coords[7]; // {x, y, z, lx, ly, lz}
            if (!readBytes(clientSocket, coords, 6 * sizeof(float)))
            {
                perror("Failed to read 6 floats for SET_CAMERA command.\n");
                sendBoolValue(clientSocket, false);
                return -1;
            }

            std::cout << "Updating camera to position ("
                      << coords[0] << ", " << coords[1] << ", " << coords[2] << ") "
                      << "and look-at ("
                      << coords[3] << ", " << coords[4] << ", " << coords[5] << ")"
                      << std::endl;

            vec3 new_pos = make_vec3(coords[0], coords[1], coords[2]);
            vec3 new_lookat = make_vec3(coords[3], coords[4], coords[5]);

            // Update the radiation model
            if (radiation_model)
            {
                radiation_model->setCameraPosition(m_config.cameraName, new_pos);
                radiation_model->setCameraLookat(m_config.cameraName, new_lookat);
                radiation_model->runCamera("simple");
            }

            // Update the visualizer
            visualizer.setCameraPosition(new_pos, new_lookat);
            visualizer.plotFastUpdate(true);

            // Acknowledge the command
            sendBoolValue(clientSocket, true);
            return 0;
        }

        if (command == "GET_FRUITS_LOCATIONS")
        {
            std::vector<std::vector<float>> fruits_locations;
            int fruit_id = 0;
            std::vector<uint> UUIDs_plants = plantarchitecture.getAllPlantIDs();
            for (int i = 0; i < UUIDs_plants.size(); i++)
            {
                std::vector<uint> fruit_obj_UUIDs = plantarchitecture.getPlantFruitObjectIDs(UUIDs_plants[i]);
                for (int j = 0; j < fruit_obj_UUIDs.size(); j++)
                {
                    vec3 min_corner, max_corner;
                    plantarchitecture.getContextPtr()->getObjectBoundingBox(fruit_obj_UUIDs[j], min_corner, max_corner);
                    vec3 center = (min_corner + max_corner) / 2.0f;
                    vec3 size = (max_corner - min_corner) * 2.0f;
                    plantarchitecture.getContextPtr()->getObjectData(fruit_obj_UUIDs[j], "fruit_id", fruit_id);
                    
                    std::vector<float> fruit_location;
                    fruit_location.push_back(i);
                    fruit_location.push_back(fruit_id);
                    fruit_location.push_back(center.x);
                    fruit_location.push_back(center.y);
                    fruit_location.push_back(center.z);
                    fruit_location.push_back(size.x);
                    fruit_location.push_back(size.y);
                    fruit_location.push_back(size.z);

                    fruits_locations.push_back(fruit_location);
                }
            }

            if (!sendFloat2DArray(clientSocket, fruits_locations))
            {
                perror("Failed to send fruits locations");
                return -1;
            }
            return 0;
        }

        std::cerr << "Unknown command: " << command << std::endl;
        return 0;
    }

private:
    void setNonBlocking(int socket_fd)
    {
        int flags = fcntl(socket_fd, F_GETFL, 0);
        if (flags == -1)
        {
            perror("Failed to get socket flags");
            return;
        }

        if (fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK) == -1)
        {
            perror("Failed to set socket non-blocking");
        }
    }

    bool sendString(int clientSocket, const std::string &str)
    {
        uint8_t msgType = static_cast<uint8_t>(MessageTypeClass::STRING);
        if (!sendBytes(clientSocket, &msgType, 1))
            return false;

        uint32_t msgSize = static_cast<uint32_t>(str.size());
        if (!sendBytes(clientSocket, &msgSize, 4))
            return false;

        if (!sendBytes(clientSocket, str.data(), str.size()))
            return false;

        return true;
    }

    bool sendFloatValue(int clientSocket, float value)
    {
        uint8_t msgType = static_cast<uint8_t>(MessageTypeClass::FLOAT);
        if (!sendBytes(clientSocket, &msgType, 1))
            return false;

        uint32_t msgSize = 1;
        if (!sendBytes(clientSocket, &msgSize, 4))
            return false;

        if (!sendBytes(clientSocket, &value, sizeof(float)))
            return false;

        return true;
    }

    bool sendIntValue(int clientSocket, int value)
    {
        uint8_t msgType = static_cast<uint8_t>(MessageTypeClass::INT);
        if (!sendBytes(clientSocket, &msgType, 1))
            return false;

        uint32_t msgSize = 1;
        if (!sendBytes(clientSocket, &msgSize, 4))
            return false;

        if (!sendBytes(clientSocket, &value, sizeof(int)))
            return false;

        return true;
    }

    bool sendBoolValue(int clientSocket, bool value)
    {
        uint8_t msgType = static_cast<uint8_t>(MessageTypeClass::BOOL);
        if (!sendBytes(clientSocket, &msgType, 1))
            return false;

        uint32_t msgSize = 1;
        if (!sendBytes(clientSocket, &msgSize, 4))
            return false;

        if (!sendBytes(clientSocket, &value, sizeof(bool)))
            return false;

        return true;
    }

    bool sendFloat1DArray(int clientSocket, const std::vector<float> &array)
    {
        uint8_t msgType = static_cast<uint8_t>(MessageTypeClass::FLOAT_1D_ARRAY);
        if (!sendBytes(clientSocket, &msgType, 1))
            return false;

        uint32_t msgSize = static_cast<uint32_t>(array.size());
        if (!sendBytes(clientSocket, &msgSize, 4))
            return false;

        if (!sendBytes(clientSocket, array.data(), sizeof(float) * msgSize))
            return false;

        return true;
    }

    bool sendInt1DArray(int clientSocket, const std::vector<int> &array)
    {
        uint8_t msgType = static_cast<uint8_t>(MessageTypeClass::INT_1D_ARRAY);
        if (!sendBytes(clientSocket, &msgType, 1))
            return false;

        uint32_t msgSize = static_cast<uint32_t>(array.size());
        if (!sendBytes(clientSocket, &msgSize, 4))
            return false;

        if (!sendBytes(clientSocket, array.data(), sizeof(int) * msgSize))
            return false;

        return true;
    }

    bool sendUInt81DArray(int clientSocket, const std::vector<uint8_t> &array)
    {
        uint8_t msgType = static_cast<uint8_t>(MessageTypeClass::UINT8_1D_ARRAY);
        if (!sendBytes(clientSocket, &msgType, 1))
            return false;

        uint32_t msgSize = static_cast<uint32_t>(array.size());
        if (!sendBytes(clientSocket, &msgSize, 4))
            return false;

        if (!sendBytes(clientSocket, array.data(), sizeof(uint8_t) * msgSize))
            return false;

        return true;
    }

    bool sendUInt161DArray(int clientSocket, const std::vector<uint16_t> &array)
    {
        uint8_t msgType = static_cast<uint8_t>(MessageTypeClass::UINT16_1D_ARRAY);
        if (!sendBytes(clientSocket, &msgType, 1))
            return false;

        uint32_t msgSize = static_cast<uint32_t>(array.size());
        if (!sendBytes(clientSocket, &msgSize, 4))
            return false;

        if (!sendBytes(clientSocket, array.data(), sizeof(uint16_t) * msgSize))
            return false;

        return true;
    }

    bool sendFloat2DArray(int clientSocket, const std::vector<std::vector<float>> &array2D)
    {
        uint8_t msgType = static_cast<uint8_t>(MessageTypeClass::FLOAT_2D_ARRAY);
        if (!sendBytes(clientSocket, &msgType, 1))
            return false;

        uint32_t numRows = static_cast<uint32_t>(array2D.size());
        if (!sendBytes(clientSocket, &numRows, 4))
            return false;

        uint32_t numCols = array2D.empty() ? 0 : static_cast<uint32_t>(array2D[0].size());
        if (!sendBytes(clientSocket, &numCols, 4))
            return false;

        for (const auto &array : array2D)
        {
            if (array.size() != numCols)
            {
                perror("All rows must have the same number of columns");
                return false;
            }
            if (!sendBytes(clientSocket, array.data(), sizeof(float) * numCols))
                return false;
        }

        return true;
    }

    bool sendBytes(int clientSocket, const void *buffer, size_t bufferSize)
    {
        size_t bytesSent = 0;

        while (bytesSent < bufferSize)
        {
            const char *currentPosition = static_cast<const char *>(buffer) + bytesSent;

            ssize_t bytesTransferred = send(clientSocket, currentPosition, bufferSize - bytesSent, 0);
            if (bytesTransferred < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    continue;
                }
                perror("Failed to send data");
                return false;
            }
            bytesSent += static_cast<size_t>(bytesTransferred);
        }
        return true;
    }

    bool readBytes(int socket_fd, void *buffer, size_t totalBytes)
    {
        size_t bytesRead = 0;
        char *bufPtr = static_cast<char *>(buffer);

        while (bytesRead < totalBytes)
        {
            ssize_t ret = read(socket_fd, bufPtr + bytesRead, totalBytes - bytesRead);
            if (ret > 0)
            {
                bytesRead += static_cast<size_t>(ret);
            }
            else if (ret == 0)
            {
                return false;
            }
            else
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    continue;
                }
                perror("read() error");
                return false;
            }
        }
        return true;
    }

private:
    int m_port;
    int m_serverFd;
    bool m_useNonBlocking;

    Config m_config;

    enum class MessageTypeClass
    {
        STRING = 0,
        FLOAT = 1,
        INT = 2,
        BOOL = 3,

        FLOAT_1D_ARRAY = 11,
        INT_1D_ARRAY = 12,
        UINT8_1D_ARRAY = 13,
        UINT16_1D_ARRAY = 14,

        FLOAT_2D_ARRAY = 21,
    };
};
