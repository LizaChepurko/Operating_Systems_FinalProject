#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <sys/wait.h>

#include "Patterns.cpp"

#define PORT 8080
#define MAX_CLIENTS 30
#define BUFFER_SIZE 8192

/**
 * @brief The MSTServer class handles client requests to perform
 * operations on a graph, such as adding or removing edges, and
 * solving the Minimum Spanning Tree (MST) problem using a separate process.
 * Plus pipes for communication with the Graph process
 */

class MSTServer
{
private:
    int graphCommandPipeFd[2]; // Pipe for sending commands to the Graph process
    int graphResultPipeFd[2];  // Pipe for receiving results from the Graph process
    pid_t graphPid;
    ActiveObject pipelineExecutor;
    LeaderFollowerPool lfPool;

    /**
     * @brief Starts the Graph process, setting up pipes for inter-process communication.
     *        This process handles graph-related operations like addition/removal of edges
     *        and solving the MST.
     */
    void startGraphProcess()
    {
        // Create command pipe
        if (pipe(graphCommandPipeFd) == -1)
        {
            perror("pipe (command)");
            exit(EXIT_FAILURE);
        }

        // Create result pipe
        if (pipe(graphResultPipeFd) == -1)
        {
            perror("pipe (result)");
            exit(EXIT_FAILURE);
        }

        graphPid = fork();
        if (graphPid == -1)
        {
            perror("fork");
            exit(EXIT_FAILURE);
        }
        // Child process
        else if (graphPid == 0)
        {
            // Close unused ends of the pipes
            close(graphCommandPipeFd[1]);
            close(graphResultPipeFd[0]);

            // Redirect stdin and stdout to the pipes
            dup2(graphCommandPipeFd[0], STDIN_FILENO);
            dup2(graphResultPipeFd[1], STDOUT_FILENO);
            dup2(graphResultPipeFd[1], STDERR_FILENO);

            // Close the read and write end of the command pipe
            close(graphCommandPipeFd[0]);
            close(graphResultPipeFd[1]);
            // Execute the Graph process
            execl("./Graph", "Graph", nullptr);
            perror("execl");
            exit(EXIT_FAILURE);
        }
        // Parent process
        else
        {
            // Close unused ends of the pipes
            close(graphCommandPipeFd[0]);
            close(graphResultPipeFd[1]);
        }
    }

public:
    /**
     * Constructs an MSTServer with a specified number of threads
     * for handling client requests and graph operations.
     *
     * @param numThreads Number of threads in the leader-follower pool.
     */
    MSTServer(size_t numThreads) : lfPool(numThreads)
    {
        startGraphProcess();
    }
    /**
     * Handles incoming client connections and processes their requests.
     *
     * @param clientSocket Socket descriptor for the connected client.
     */
    void handleClient(int clientSocket)
    {
        char buffer[BUFFER_SIZE] = {0};
        while (true)
        {
            memset(buffer, 0, BUFFER_SIZE);
            int valread = read(clientSocket, buffer, BUFFER_SIZE);
            if (valread <= 0)
                break;

            std::string request(buffer);
            std::cout << "Server: Received request from client: " << request << std::endl;
            std::istringstream iss(request);
            std::string command;
            iss >> command;

            if (command == "ADD_GRAPH")
            {
                std::string result = executeCommand(request);
                if (result.empty() || result == "No response from Graph process")
                {
                    result = "Graph added successfully\n";
                }
                std::cout << "Sending to client: " << result << std::endl;
                send(clientSocket, result.c_str(), result.length(), 0);
            }
            else if (command == "ADD_EDGE")
            {
                std::string result = executeCommand(request);
                if (result.empty() || result == "No response from Graph process")
                {
                    result = "Edge added successfully\n";
                }
                std::cout << "Sending to client: " << result << std::endl;
                send(clientSocket, result.c_str(), result.length(), 0);
            }
            else if (command == "REMOVE_EDGE")
            {
                std::string result = executeCommand(request);
                if (result.empty() || result == "No response from Graph process")
                {
                    result = "Edge removed successfully\n";
                }
                std::cout << "Sending to client: " << result << std::endl;
                send(clientSocket, result.c_str(), result.length(), 0);
            }
            else if (command == "REMOVE_GRAPH")
            {
                std::string result = executeCommand(request);
                if (result.empty() || result == "No response from Graph process")
                {
                    result = "Graph removed successfully\n";
                }
                std::cout << "Sending to client: " << result << std::endl;
                send(clientSocket, result.c_str(), result.length(), 0);
            }
            else if (command == "SOLVE_MST_PIPELINE")
            {
                std::cout << "Server: Queuing task for Pipeline..." << std::endl;
                std::string solveCommand = "SOLVE_MST" + request.substr(command.length());
                auto task = [this, clientSocket, solveCommand]()
                {
                    std::string result = executeCommand(solveCommand);
                    if (result.empty() || result == "No response from Graph process")
                    {
                        result = "Error: No response from Graph process\n";
                    }
                    std::cout << "Pipeline Worker: Sending to client: " << result << std::endl;

                    // Read all results from the 'result' string
                    std::istringstream iss(result);
                    std::string line;

                    std::getline(iss, line); // Skip the "Graph X MST Results:" line
                    std::getline(iss, line); // Get Total Weight
                    std::getline(iss, line); // Get Longest Distance
                    std::getline(iss, line); // Get Average Distance
                    std::getline(iss, line); // Get Shortest Distance

                    // Send the entire result string to the client
                    send(clientSocket, result.c_str(), result.length(), 0);
                };
                pipelineExecutor.enqueue(task);
            }
            else if (command == "SOLVE_MST_LF")
            {
                std::cout << "Server: Queuing task for Leader-Follower..." << std::endl;
                std::string solveCommand = "SOLVE_MST" + request.substr(command.length());
                auto task = [this, clientSocket, solveCommand]()
                {
                    std::string result = executeCommand(solveCommand);
                    if (result.empty() || result == "No response from Graph process")
                    {
                        result = "Error: No response from Graph process\n";
                    }
                    std::cout << "Leader-Follower Thread: Sending to client: " << result << std::endl;

                    // Read all results from the 'result' string
                    std::istringstream iss(result);
                    std::string line;

                    std::getline(iss, line); // Skip the "Graph X MST Results:" line
                    std::getline(iss, line); // Get Total Weight
                    std::getline(iss, line); // Get Longest Distance
                    std::getline(iss, line); // Get Average Distance
                    std::getline(iss, line); // Get Shortest Distance

                    // Send the entire result string to the client
                    send(clientSocket, result.c_str(), result.length(), 0);
                };
                lfPool.enqueue(task);
            }
            else
            {
                std::string errorMsg = "Unknown command\n";
                std::cout << "Sending to client: " << errorMsg << std::endl;
                send(clientSocket, errorMsg.c_str(), errorMsg.length(), 0);
            }
        }
        close(clientSocket);
    }

    /**
     * Executes a command by sending it to the Graph process
     * and receiving the response from it.
     *
     * @param command The command string to be processed by the Graph process.
     * @return The response received from the Graph process.
     */
    std::string executeCommand(const std::string &command)
    {
        std::cout << "Executing command: " << command << std::endl;
        // Write command to the command pipe
        write(graphCommandPipeFd[1], (command + "\n").c_str(), command.length() + 1);

        // Read response from the result pipe
        char buffer[BUFFER_SIZE];
        std::string result;
        ssize_t bytesRead;

        while (true)
        {
            bytesRead = read(graphResultPipeFd[0], buffer, BUFFER_SIZE - 1);
            if (bytesRead > 0)
            {
                buffer[bytesRead] = '\0';
                result += buffer;

                if (result.find("\n") != std::string::npos)
                {
                    break;
                }
            }
            else if (bytesRead == 0)
            {
                break; // Reached EOF
            }
            else
            {
                perror("read error (result pipe)");
                break;
            }
        }

        std::cout << "Received result from Graph process" << result << std::endl;
        return result;
    }

    /**
     * Starts accepting client connections and handles them concurrently.
     */
    void run()
    {
        int server_fd, new_socket;
        struct sockaddr_in address;
        int opt = 1;
        int addrlen = sizeof(address);

        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
        {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
        {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(PORT);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        if (listen(server_fd, 3) < 0)
        {
            perror("listen");
            exit(EXIT_FAILURE);
        }

        std::cout << "Server is running on port " << PORT << std::endl;

        while (true)
        {
            if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
            {
                perror("accept");
                continue;
            }
            // Handle each client connection in a separate thread
            std::thread clientThread(&MSTServer::handleClient, this, new_socket);
            clientThread.detach();
        }
    }
    /**
     * Destructor to clean up resources and terminate the Graph process.
     */
    ~MSTServer()
    {
        close(graphCommandPipeFd[1]);
        close(graphResultPipeFd[0]);
        waitpid(graphPid, nullptr, 0);
    }
};