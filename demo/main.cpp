#include <fstream>

#include "viewer/Renderer.h"
#include "tracker/Tracker.h"
#include "CloudManager.h"
#include <boost/filesystem.hpp>

int main(int argc, char *argv[])
{
    int64_t freq = 100;            // Frequency of the thread dedicated to process the point cloud
    std::string log_path = "/home/ilpiova/Autonomous/assignment/assignment_2/assignment_2/build/log";  // TODO: define the path to the log folder

    if (!boost::filesystem::exists(log_path) || !boost::filesystem::is_directory(log_path))
        {
            std::cerr << "ERRORE: La cartella '" << log_path << "' non esiste o non è una cartella valida.\n";
            return 1;
        }

    // Init renderer
    viewer::Renderer renderer;
    renderer.initCamera(viewer::CameraAngle::XY);
    renderer.clearViewer();

    // Instantiate the tracker
    Tracker tracker;

    // Spawn the thread that process the point cloud and performs the clustering
    CloudManager lidar_cloud(log_path, freq, renderer);
    std::thread t(&CloudManager::startCloudManager, &lidar_cloud); //invoco l'esecuzione del metodo startCloudManager della classe CloudManager in un thread separato, passando l'oggetto lidar_cloud come argomento. Questo permette di eseguire il metodo in parallelo al thread principale del main.

    while (true)
    {
        // Clear the render
        renderer.clearViewer();

        while (!lidar_cloud.new_measurement) //attendo che il thread lidar_cloud segnali che ci sono nuovi dati disponibili (new_measurement è un flag booleano che indica se c'è un nuovo frame da elaborare). Se non ci sono nuovi dati, il thread principale entra in un ciclo di attesa.
            ; // wait for new data (we will execute the following code each 100ms)

        // Prendo il frame corrente che è stato processato all'interno di CloudManager::startCloudManager e ne estrepolo le sue informazioni (centroidi, bounding box, point cloud) per poterle renderizzare e trakkare gli oggetti rilevati.   
        lidar_cloud.mtxData.lock(); //mtxData è un mutex che serve a proteggere l'accesso ai dati condivisi tra il thread principale e il thread del CloudManager. In questo caso, viene utilizzato per garantire che i dati del point cloud e delle informazioni associate siano letti in modo sicuro senza conflitti di accesso tra il thread del main e il thread lidar_cloud.
        auto cloud = lidar_cloud.getCloud();
        auto color = lidar_cloud.getColor();
        auto boxes = lidar_cloud.getBoxes();
        auto centroids_x = lidar_cloud.getCentroidsX();
        auto centroids_y = lidar_cloud.getCentroidsY();
        lidar_cloud.new_measurement = false;
        lidar_cloud.mtxData.unlock();

        // renderizzo la point cloud e le bounding box dei cluster rilevati 
        renderer.renderPointCloud(cloud, "pointCloud", color);
        for (size_t i = 0; i < boxes.size(); ++i)
            renderer.renderBox(boxes[i], i);

        // eseguo il tracking dei cluster rilevati passando in input la lista dei centroidi e lo stato del sensore lidar (attivo o meno).
        tracker.track(centroids_x, centroids_y, renderer.getLidarStatus());

        // retrieve tracklets and render the trackers
        auto tracks = tracker.getTracks();
        for (size_t i = 0; i < tracks.size(); ++i)
        {
            renderer.addCircle(tracks[i].getX(), tracks[i].getY(), tracks[i].getId());
            renderer.addText(tracks[i].getX() + 0.01, tracks[i].getY() + 0.01, tracks[i].getId());
        }

        std::pair<int, double> longest_track = tracker.getLongestTrack();

        if (longest_track.first != -1) {
            std::cout << "Track con percorso piu' lungo:" << std::endl;
            std::cout << "  - ID: " << longest_track.first << std::endl;
            std::cout << "  - Lunghezza: " << longest_track.second << " m" << std::endl;
        } else {
            std::cout << "Nessuna traccia registrata." << std::endl;
        }

        renderer.spinViewerOnce(); 
    }

    t.join(); // Wait for the thread to finish (this line will never be reached in this infinite loop, but it's good practice to include it for proper thread management)
    return 0; // Return 0 to indicate successful execution
}
