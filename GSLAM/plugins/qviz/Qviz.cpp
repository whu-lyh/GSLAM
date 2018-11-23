#include "GSLAM/core/Application.h"
#include <QApplication>
#include "MainWindow.h"

class QVisualizer: public GSLAM::Application
{
public:
    QVisualizer():GSLAM::Application("Qviz"){}
    ~QVisualizer(){
        sub_dataset_status.shutdown();
        work_thread.join();
    }

    virtual GSLAM::Messenger init(GSLAM::Svar configuration){
        config=configuration;

        pub_gui=messenger.advertise<std::string>("gui",0);
        sub_gui=messenger.subscribe("visualize",0,&QVisualizer::process,this);
        sub_dataset_status=messenger.subscribe("dataset/status",0,
                           &QVisualizer::datasetStatusUpdated,this);

        if(!configuration.Get<bool>("help"))
            work_thread=std::thread(&QVisualizer::gui_thread,this);
        return messenger;
    }

    void gui_thread(){
        QApplication app(config.GetInt("argc"),
                         (char**)config.GetPointer("argv"));

        SPtr<GSLAM::MainWindow> mainWindow(new GSLAM::MainWindow());
        mainWindow->pub_gui=pub_gui;
        auto dataset=svar.GetString("Dataset");
        mainWindow->show();
        this->mainWindow=mainWindow;

        auto publishers=GSLAM::Messenger::instance().getPublishers();
        for(auto it:publishers)
            for(const GSLAM::Publisher& pub:it.second){
                process(pub);
            }
        if(!dataset.empty())
            mainWindow->slotStartDataset(dataset.c_str());

        app.exec();
        mainWindow.reset();
        running_=false;
        svar.GetInt("ShouldStop")=1;
    }

    void datasetStatusUpdated(const int& status){
        if(mainWindow) mainWindow->datasetStatusUpdated(status);
    }

    void process(const GSLAM::Publisher& pub){
        if(mainWindow) mainWindow->tryVisualize(pub);
    }

    std::thread       work_thread;
    GSLAM::Svar       config;
    GSLAM::Messenger  messenger;
    GSLAM::Publisher  pub_gui;
    GSLAM::Subscriber sub_gui,sub_dataset_status;
    SPtr<GSLAM::MainWindow> mainWindow;
};

REGISTER_APPLICATION(QVisualizer);
