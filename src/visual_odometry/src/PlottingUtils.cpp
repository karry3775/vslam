#include <visual_odometry/PlottingUtils.h>


namespace vslam{

void drawCube(){

    pangolin::CreateWindowAndBind("Main", 640, 480);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
}

void drawCubeOffScreen(){

    static const int w = 640;
    static const int h = 480;

    pangolin::CreateWindowAndBind("Main", w, h, pangolin::Params({{"scheme", "headless"}}));
    glEnable(GL_DEPTH_TEST);

    // Define the Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(w, h, 420, 420, 320, 240, 0.2, 100),
        pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY)
    );

    // Create Interative View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -float(w) / float(h))
            .SetHandler(&handler);

    pangolin::SaveWindowOnRender("window");

    // create a frame buffer objecct with colour and depth buffer
    pangolin::GlTexture color_buffer(w, h);
    pangolin::GlRenderBuffer depth_buffer(w, h);
    pangolin::GlFramebuffer fbo_buffer(color_buffer, depth_buffer);
    fbo_buffer.Bind();

    //Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);

    // Render OpenGl Cube
    pangolin::glDrawColouredCube();

    // Swap frame and Process Events
    pangolin::FinishFrame();

    fbo_buffer.Unbind();
    // download and save the color buffer
    color_buffer.Save("fbo.png", false);

    pangolin::QuitAll();


}

void drawSimpleScene(){
    // Create a pangolin window and bind
    pangolin::CreateWindowAndBind("Main", w_, h_);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100), /**w, h, fu, fv, u0, v0, znear, zfar**/
        pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY)
    );

    pangolin::Renderable tree;
    tree.Add( std::make_shared<pangolin::Axis>() );

    // Create Interatice View in window
    pangolin::SceneHandler handler(tree, s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, (-w_*1.0) / (h_*1.0))
            .SetHandler(&handler);

    d_cam.SetDrawFunction([&](pangolin::View& view){
        view.Activate(s_cam);
        tree.Render();
    });

    while( !pangolin::ShouldQuit() ) {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
    

}

void drawSimplePlot() {
    // Create OpenGl window in single line
    pangolin::CreateWindowAndBind("Main", w_, h_);

    // Data logger object
    pangolin::DataLog log;

    // Optionally add named labels
    std::vector<std::string> labels;
    labels.push_back(std::string("sin(t)"));
    labels.push_back(std::string("cos(t)"));
    labels.push_back(std::string("sin(t) + cos(t)"));
    log.SetLabels(labels);

    const float tinc = 0.01f; // time increment

    // OpenGl "view" of data. We might have many views of the same data
    pangolin::Plotter plotter(&log, 0.0f, 4.0f*(float)M_PI/tinc, -2.0f, 2.0f, (float)M_PI/(4.0f*tinc), 0.5f);
    plotter.SetBounds(0.0, 1.0, 0.0, 1.0);
    plotter.Track("$i");

    // Add some sample annotation to the plot
    plotter.AddMarker( pangolin::Marker::Vertical, -1000, pangolin::Marker::LessThan, 
                       pangolin::Colour::Blue().WithAlpha(0.2f) );
    plotter.AddMarker( pangolin::Marker::Horizontal, 100, pangolin::Marker::GreaterThan,
                       pangolin::Colour::Red().WithAlpha(0.2f) );
    plotter.AddMarker( pangolin::Marker::Horizontal, 10, pangolin::Marker::Equal, pangolin::Colour::Green()
                       .WithAlpha( 0.2f) );

    pangolin::DisplayBase().AddDisplay(plotter);

    float t = 0;

    // Default hooks for existing (Esc) and full sreen (tab)
    while ( !pangolin::ShouldQuit() ){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        log.Log(sin(t), cos(t), sin(t) + cos(t) );
        t += tinc;

        // Render graph, Swap frames and Process Events
        pangolin::FinishFrame();
    }

}

void drawCurrentCamera(pangolin::OpenGlMatrix &Twc) {

    const float w  = 10;
    const float h = 0.75 * w;
    const float z = 0.6 * w;

    glPushMatrix();
    glMultMatrixd(Twc.m);

    glLineWidth(2);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();

}

void drawFakeMap() {
    // Create a pangolin window
    pangolin::CreateWindowAndBind("VSLA : Map viewer", w_, h_);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Camera Render object (for view / scene browsing)
   pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    // Add named OpenGl viewport to window and provide 3D Handle
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    pangolin::OpenGlMatrix Twc;//  what the fuck is this
    Twc.SetIdentity(); // lets leave it at this

    while( !pangolin::ShouldQuit() ) {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        s_cam.Follow(Twc);
        d_cam.Activate(s_cam);
        drawCurrentCamera(Twc);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        pangolin::FinishFrame();

        // update Twc
        Twc.m[11] += 0.1;
        
    }

}
} // namespace vslam