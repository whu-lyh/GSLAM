#include <GSLAM/core/Application.h>
#include <GSLAM/core/Timer.h>
#include <GSLAM/core/Dataset.h>
#include <list>
#include <sstream>

using namespace std;
using namespace GSLAM;

class ApplicationDataset:public Application{
public:
    ApplicationDataset():Application("play"){}

    Messenger init(Svar configuration){
        auto dataset=configuration.Arg<std::string>("dataset","","The dataset going to play.");
        _player=std::make_shared<DatasetPlayer>(Dataset(dataset),_messenger,configuration);
        return _messenger;
    }

    std::shared_ptr<DatasetPlayer> _player;
    Messenger                      _messenger;
};

REGISTER_BUILDIN_APPLICATION(ApplicationDataset,play);

int main(int argc,char** argv)
{
    auto messenger=GSLAM::Messenger::instance();
    GSLAM::ScopedTimer tm("Main");
    auto unParsed=svar.ParseMain(argc,argv);
    auto datasetPath=svar.Arg<std::string>("Dataset","","The Dataset location with extesion.");
    bool qviz=svar.Arg<bool>("qviz",true,"Use buildin Qt visualizer or not.");

    if(datasetPath.size()&&qviz&&unParsed.empty()){
        unParsed.push_back("qviz");
    }

    GSLAM::Publisher pub_visualize=messenger.advertise<GSLAM::Publisher>("visualize",0);

    std::list<GSLAM::ApplicationPtr> apps;
    for(std::string& appname:unParsed){
        GSLAM::ApplicationPtr app=GSLAM::Application::create(appname);
        if(!app){
            LOG(ERROR)<<"Unable to open application "<<appname
                     <<", are you inputting the right name?";
            continue;
        }

        GSLAM::Messenger msg=app->init(svar);
        if(svar.GetInt("help")){
            std::stringstream sst;
            sst<<"Usage:\n gslam "<<appname
              <<" [--help] [-conf configure_file] [-arg_name arg_value]...\n\n"
            <<msg.introduction()<<"\n\n"
            <<"Supported by GSLAM "<<app->gslam_version()<<". ";

            svar.setUsage(sst.str());
            std::cerr<<svar.help();
            return 0;
        }

        GSLAM::Messenger::instance().join(msg);
        auto publishers=msg.getPublishers();
        for(auto it:publishers)
            for(const GSLAM::Publisher& pub:it.second){
                pub_visualize.publish(pub);
            }
        apps.push_back(app);
    }

    if(svar.Get<bool>("help")||apps.empty()){
        std::cerr<<svar.help();
        return 0;
    }

    int& shouldStop=svar.GetInt("ShouldStop",0);

    while(!apps.empty()&&!shouldStop){
        for(auto it=apps.begin();it!=apps.end();it++){
            if(!(*it)->isRunning()){
                LOG(INFO)<<"Application "<<(*it)->name()<<" stoped.";
                it=apps.erase(it);
            }
        }
        GSLAM::Rate::sleep(0.1);
    }

    return 0;
}
