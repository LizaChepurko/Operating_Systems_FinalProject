#include "MSTServer.cpp"
#include "Patterns.cpp"

/**
 * Entry point for the server application.
 * 
 * This function creates and runs an instance of the MSTServer,
 * which handles incoming connections and processes graph-related
 * requests using a Leader-Follower thread pool.
 * 
 * @return int Exit status of the program.
 */

int main()
{
     // Create server with a specified number of threads in the Leader-Follower pool
    MSTServer server(5); 
    server.run(); // Start the server to begin listening for and processing client requests
    return 0;
}