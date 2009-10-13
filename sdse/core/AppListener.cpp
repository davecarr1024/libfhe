#include "AppListener.h"
#include "App.h"

using namespace sdse;

AppListener::AppListener(App* _app) :
    app(0) {
    attach(_app);
}

AppListener::AppListener() : app(0) {}

void AppListener::attach(App* _app) {
    if (_app != app) {
        detach();
        app = _app;
        app->addListener(this);
    }
}

void AppListener::detach() {
    if (app) {
        App* _app = app;
        app = 0;
        _app->removeListener(this);
    }
}
