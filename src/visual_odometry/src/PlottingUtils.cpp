 #include <visual_odometry/PlottingUtils.h>


 // source : https://stackoverflow.com/questions/765408/free-easy-way-to-draw-graphs-and-charts-in-c
 void plot(){
    try {
        // Python startup code
        Py_Initialize();
        PyRun_SimpleString("import signal");
        PyRun_SimpleString("signal.signal(signal.SIGINT, signal.SIG_DFL)");


        // Normal Gtk startup code
        Gtk::Main kit(0,0); // TODO : get the cmakelists fixed
        
    } catch (...) {
        std::cout << "Error occured!" ;
    }

}