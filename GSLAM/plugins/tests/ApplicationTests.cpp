#include "GSLAM/core/Application.h"
#include "gtest.h"
using namespace GSLAM;

class ApplicationTest: public GSLAM::Application
{
public:
    ApplicationTest()
        :GSLAM::Application("Tests"){}

    virtual Messenger init(Svar configuration)
    {
        svar=configuration;
        testing::InitGoogleTest(&svar.GetInt("argc"),(char**)svar.GetPointer("argv"));
        int ret=RUN_ALL_TESTS();
        running_=false;
        return Messenger();
    }
};

REGISTER_APPLICATION(ApplicationTest);
