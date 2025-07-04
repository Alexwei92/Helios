#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <netinet/tcp.h>
#include <thread>
#include <unistd.h>

#include "PlantArchitecture.h"
#include "RadiationModel.h"
#include "Visualizer.h"

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

    enum class handleCommandFlag
    {
        ERROR = -1,
        OK = 0,
        EXIT = 1,
    };

    SocketServer(int port, const Config& config)
        : m_port(port), m_config(config) {
    }

    ~SocketServer() { stop(); }

    int start(helios::Context* context_ptr,
        Visualizer& visualizer, PlantArchitecture& plantarchitecture,
        RadiationModel* radiation_model = nullptr)
    {
        if (!setupServerSocket())
        {
            return -1;
        }

        while (true)
        {
            if (!acceptAndServe(context_ptr, visualizer, plantarchitecture, radiation_model))
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
        if (setsockopt(m_serverFd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0 ||
            setsockopt(m_serverFd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) < 0)
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
        if (bind(m_serverFd, (struct sockaddr*)(&address), sizeof(address)) < 0)
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

    bool acceptAndServe(helios::Context* context_ptr,
        Visualizer& visualizer,
        PlantArchitecture& plantarchitecture,
        RadiationModel* radiation_model)
    {
        sockaddr_in address{};
        socklen_t addrLen = sizeof(address);

        int clientSocket = accept(m_serverFd, (struct sockaddr*)(&address), &addrLen);
        if (clientSocket < 0)
        {
            perror("accept() failed");
            return true;
        }

        std::cout << "Client connected. Waiting for commands..." << std::endl;

        pollfd pfd = { clientSocket, POLLIN, 0 };
        constexpr size_t BUFFER_SIZE = 4096;
        char buffer[BUFFER_SIZE];

        while (true)
        {
            int ret = poll(&pfd, 1, 100); // 100ms timeout
            if (ret < 0)
            {
                perror("poll() error");
                break;
            }
            if (ret == 0 || !pfd.revents & POLLIN)
            {
                continue;
            }

            // Receive data from the client
            ssize_t bytesRecv = recv(clientSocket, buffer, BUFFER_SIZE, 0);
            if (bytesRecv <= 0)
            {
                std::cout << "Client disconnected. Waiting for new connection..." << std::endl;
                break;
            }

            // Handle the command
            std::string_view command(buffer, static_cast<size_t>(bytesRecv));
            auto flag = handleCommand(command, clientSocket, context_ptr, visualizer, plantarchitecture, radiation_model);
            if (flag == handleCommandFlag::EXIT)
            {
                close(clientSocket);
                stop();
                return false;
            }
            if (flag == handleCommandFlag::ERROR)
            {
                break;
            }
        }

        close(clientSocket);
        return true;
    }

    handleCommandFlag handleCommand(std::string_view command, int clientSocket, helios::Context* context_ptr, Visualizer& visualizer, PlantArchitecture& plantarchitecture, RadiationModel* radiation_model)
    {
        if (command == "EXIT")
        {
            std::cout << "Exit command received. Stopping server." << std::endl;
            return handleCommandFlag::EXIT;
        }

        if (command == "GET_CAMERA_DEPTH_MAX")
        {
            if (!sendScalar(clientSocket, MessageType::FLOAT, m_config.cameraDepthMax))
            {
                perror("Failed to send max depth value");
                return handleCommandFlag::ERROR;
            }
            return handleCommandFlag::OK;
        }

        if (command == "GET_CAMERA_RESOLUTION")
        {
            std::vector<int> resolution;
            resolution.push_back(m_config.cameraHeight);
            resolution.push_back(m_config.cameraWidth);
            if (!send1DArray(clientSocket, MessageType::INT_1D_ARRAY, resolution))
            {
                perror("Failed to send camera resolution");
                return handleCommandFlag::ERROR;
            }
            return handleCommandFlag::OK;
        }

        if (command == "GET_CAMERA_FOV")
        {
            if (!sendScalar(clientSocket, MessageType::FLOAT, m_config.cameraFOV))
            {
                perror("Failed to send camera FOV");
                return handleCommandFlag::ERROR;
            }
            return handleCommandFlag::OK;
        }

        if (command == "GET_CAMERA_POSE")
        {
            std::vector<float> camera_pose = visualizer.getCameraPose();
            if (!send1DArray(clientSocket, MessageType::FLOAT_1D_ARRAY, camera_pose))
            {
                perror("Failed to send camera pose");
                return handleCommandFlag::ERROR;
            }
            return handleCommandFlag::OK;
        }

        if (command == "GET_RGB")
        {
            std::vector<uint8_t> rgb_data = visualizer.getRGBData();

            if (!send1DArray(clientSocket, MessageType::UINT8_1D_ARRAY, rgb_data))
            {
                perror("Failed to send RGB data");
                return handleCommandFlag::ERROR;
            }
            return handleCommandFlag::OK;
        }

        if (command == "GET_NORM_DEPTH" && radiation_model)
        {
            std::vector<float> depth_data = radiation_model->getDepthData(m_config.cameraName, m_config.cameraDepthMax, true);

            if (!send1DArray(clientSocket, MessageType::FLOAT_1D_ARRAY, depth_data))
            {
                perror("Failed to send normalized depth data");
                return handleCommandFlag::ERROR;
            }
            return handleCommandFlag::OK;
        }

        if (command == "GET_ACTUAL_DEPTH" && radiation_model)
        {
            std::vector<float> depth_data = radiation_model->getDepthData(m_config.cameraName, m_config.cameraDepthMax, false);

            if (!send1DArray(clientSocket, MessageType::FLOAT_1D_ARRAY, depth_data))
            {
                perror("Failed to send actual depth data");
                return handleCommandFlag::ERROR;
            }
            return handleCommandFlag::OK;
        }

        if (command == "GET_LABEL_MAP" && radiation_model)
        {
            std::vector<int> label_map = radiation_model->getPrimitiveDataLabelMap(m_config.cameraName, "class", 0);

            if (!send1DArray(clientSocket, MessageType::INT_1D_ARRAY, label_map))
            {
                perror("Failed to send label map");
                return handleCommandFlag::ERROR;
            }
            return handleCommandFlag::OK;
        }

        if (command == "GET_BBOX_2D" && radiation_model)
        {
            std::vector<std::vector<float>> bbox_2d = radiation_model->getImageBoundingBoxes_ObjectData(m_config.cameraName, "fruit_id");

            if (!send2DArray(clientSocket, MessageType::FLOAT_2D_ARRAY, bbox_2d))
            {
                perror("Failed to send 2D bounding boxes");
                return handleCommandFlag::ERROR;
            }
            return handleCommandFlag::OK;
        }

        if (command == "SET_CAMERA")
        {
            float coords[6]; // {x, y, z, lx, ly, lz}
            if (!recvBytes(clientSocket, coords, 6 * sizeof(float)))
            {
                perror("Failed to read 6 floats for SET_CAMERA command.\n");
                sendScalar(clientSocket, MessageType::BOOL, false);
                return handleCommandFlag::ERROR;
            }

            std::cout << "Updating camera to position (" << coords[0] << ", " << coords[1] << ", " << coords[2] << ") "
                << "and look-at (" << coords[3] << ", " << coords[4] << ", " << coords[5] << ")" << std::endl;

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
            sendScalar(clientSocket, MessageType::BOOL, true);
            return handleCommandFlag::OK;
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
                    context_ptr->getObjectBoundingBox(fruit_obj_UUIDs[j], min_corner, max_corner);
                    vec3 center = (min_corner + max_corner) / 2.0f;
                    vec3 size = (max_corner - min_corner) * 2.0f;
                    context_ptr->getObjectData(fruit_obj_UUIDs[j], "fruit_id", fruit_id);

                    std::vector<float> fruit_location(8);
                    fruit_location[0] = static_cast<float>(i);
                    fruit_location[1] = static_cast<float>(fruit_id);
                    fruit_location[2] = center.x;
                    fruit_location[3] = center.y;
                    fruit_location[4] = center.z;
                    fruit_location[5] = size.x;
                    fruit_location[6] = size.y;
                    fruit_location[7] = size.z;

                    fruits_locations.push_back(fruit_location);
                }
            }

            if (!send2DArray(clientSocket, MessageType::FLOAT_2D_ARRAY, fruits_locations))
            {
                perror("Failed to send fruits locations");
                return handleCommandFlag::ERROR;
            }
            return handleCommandFlag::OK;
        }

        // if (command == "GET_RGB_NORM_DEPTH" && radiation_model)
        // {
        //     std::vector<uint8_t> rgb_data = visualizer.getRGBData();
        //     std::vector<float> depth_data = radiation_model->getDepthData(m_config.cameraName, m_config.cameraDepthMax, true);

        //     struct BlockHeader { uint8_t msgType; uint32_t msgSize; } __attribute__((packed));
        //     std::array<BlockHeader, 2> headers = { {
        //         { static_cast<uint8_t>(MessageType::UINT8_1D_ARRAY), static_cast<uint32_t>(rgb_data.size()) },
        //         { static_cast<uint8_t>(MessageType::FLOAT_1D_ARRAY), static_cast<uint32_t>(depth_data.size()) }
        //     } };

        //     iovec payload[] = {
        //         { headers.data(), sizeof(headers) },
        //         { const_cast<uint8_t*>(rgb_data.data()), rgb_data.size() },
        //         { const_cast<float*>(depth_data.data()), depth_data.size() }
        //     };

        //     if (writev(clientSocket, payload, 3) != ssize_t(sizeof(headers) + rgb_data.size() + depth_data.size()))
        //     {
        //         return handleCommandFlag::ERROR;
        //     }
        //     return handleCommandFlag::OK;
        // }

        // if (command == "GET_RGB_NORM_DEPTH_LABEL" && radiation_model)
        // {
        //     std::vector<uint8_t> rgb_data = visualizer.getRGBData();
        //     std::vector<float> depth_data = radiation_model->getDepthData(m_config.cameraName, m_config.cameraDepthMax, true);
        //     std::vector<int> label_map = radiation_model->getPrimitiveDataLabelMap(m_config.cameraName, "class", 0);

        //     struct BlockHeader { uint8_t msgType; uint32_t msgSize; } __attribute__((packed));
        //     std::array<BlockHeader, 3> headers = {
        //         { static_cast<uint8_t>(MessageType::UINT8_1D_ARRAY), static_cast<uint32_t>(rgb_data.size()) },
        //         { static_cast<uint8_t>(MessageType::FLOAT_1D_ARRAY), static_cast<uint32_t>(depth_data.size()) },
        //         { static_cast<uint8_t>(MessageType::INT_1D_ARRAY), static_cast<uint32_t>(label_map.size()) }
        //     };

        //     iovec payload[] = {
        //         { headers.data(), sizeof(headers) },
        //         { const_cast<uint8_t*>(rgb_data.data()), rgb_data.size() },
        //         { const_cast<float*>(depth_data.data()), depth_data.size() },
        //         { const_cast<int*>(label_map.data()), label_map.size() }
        //     };

        //     if (writev(clientSocket, payload, 4) != ssize_t(sizeof(headers) + rgb_data.size() + depth_data.size() + label_map.size()))
        //     {
        //         return handleCommandFlag::ERROR;
        //     }
        //     return handleCommandFlag::OK;
        // }

        std::cerr << "Unknown command: " << command << std::endl;
        return handleCommandFlag::ERROR;
    }

private:
    enum class MessageType
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

    template<typename T>
    bool sendScalar(int socket_fd, MessageType msgType, T value)
    {
        uint32_t msgSize = 1;
        iovec payload[] = { {&msgType, 1}, {&msgSize, 4}, {&value, sizeof(T)} };
        return writev(socket_fd, payload, 3) == ssize_t(1 + 4 + sizeof(T));
    }

    template<typename T>
    bool send1DArray(int socket_fd, MessageType msgType, const std::vector<T>& array)
    {
        uint32_t msgSize = static_cast<uint32_t>(array.size());
        iovec payload[] = { {&msgType, 1}, {&msgSize, 4}, {const_cast<T*>(array.data()), sizeof(T) * msgSize} };
        return writev(socket_fd, payload, 3) == ssize_t(1 + 4 + sizeof(T) * msgSize);
    }

    template<typename T>
    bool send2DArray(int socket_fd, MessageType msgType, const std::vector<std::vector<T>>& array2D)
    {
        uint32_t nRows = static_cast<uint32_t>(array2D.size());
        uint32_t nCols = array2D.empty() ? 0 : static_cast<uint32_t>(array2D[0].size());

        std::vector<iovec> payload;
        payload.reserve(3 + nRows);

        payload.push_back({ &msgType, 1 });
        payload.push_back({ &nRows, 4 });
        payload.push_back({ &nCols, 4 });

        for (const auto& row : array2D)
        {
            payload.push_back({ const_cast<T*>(row.data()), nCols * sizeof(T) });
        }

        return writev(socket_fd, payload.data(), payload.size()) ==
            static_cast<ssize_t>(1 + 4 + 4 + nRows * nCols * sizeof(T));
    }

    bool recvBytes(int socket_fd, void* buffer, size_t n)
    {
        return recv(socket_fd, buffer, n, MSG_WAITALL) == static_cast<ssize_t>(n);
    }

private:
    int m_port;
    int m_serverFd = -1;

    Config m_config;
};
