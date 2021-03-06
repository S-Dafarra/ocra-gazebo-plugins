#include "OcraTaskWidget.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(OcraTaskWidget)

/////////////////////////////////////////////////
OcraTaskWidget::OcraTaskWidget()
: GUIPlugin()
, tasksDisplayed(false)
{
    initializeGui();

}

/////////////////////////////////////////////////
OcraTaskWidget::~OcraTaskWidget()
{
    delete informUserLabel;
    delete taskButtons;
    delete buttonGroupLayout;
}

void OcraTaskWidget::initializeGui()
{
    // Set the frame background and foreground colors
    // this->setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

    // Create the main layout
    QVBoxLayout *mainLayout = new QVBoxLayout;
    this->setLayout(mainLayout);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    topLayout = new QHBoxLayout();
    mainLayout->addLayout(topLayout);
    topLayout->setContentsMargins(0, 0, 0, 0);

    buttonGroupLayout = new QVBoxLayout();
    mainLayout->addLayout(buttonGroupLayout);
    topLayout->setContentsMargins(0, 0, 0, 0);

    QPushButton *button = new QPushButton(tr("O.C.R.A."));
    button->resize(60, 30);
    connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));
    topLayout->addWidget(button);

    informUserLabel = new QLabel();
    topLayout->addWidget(informUserLabel);
    informUserLabel->hide();


    taskButtons = new QButtonGroup();
    connect(taskButtons, SIGNAL(buttonClicked(int)), this, SLOT(addTaskFrames(int)));


    // Position and resize this widget
    this->move(10, 10);
    // this->resize(300, 60);
    this->setSizePolicy(QSizePolicy(QSizePolicy::Policy::MinimumExpanding,QSizePolicy::Policy::MinimumExpanding) );

    // Create a node for transportation
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();
    this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
}

/////////////////////////////////////////////////
void OcraTaskWidget::OnButton()
{
    if (tasksDisplayed) {
        hideTaskList();
    } else {
        showTaskList();
    }
}

void OcraTaskWidget::showTaskList()
{
    getTaskList();
    if (taskNames.empty()) {
        showUserInformation("No tasks found.");
        return;
    } else {
        for (std::string name : taskNames) {
            taskActivationVector.push_back(false);
            std::cout << "Found task: " << name << std::endl;
            QPushButton* btn = new QPushButton(name.c_str());
            if (taskRelayMap.find(name)!=taskRelayMap.end()) {
                btn->setEnabled(false);
            }
            buttonGroupLayout->addWidget(btn);
            taskButtons->addButton(btn);
        }
        QPushButton *button = new QPushButton(tr("Reconnect"));
        // button->resize(60, 30);
        connect(button, SIGNAL(clicked()), this, SLOT(reconnectRelays()));
        topLayout->addWidget(button);
        // buttonFrame->show();
        tasksDisplayed = true;
    }
}

void OcraTaskWidget::reconnectRelays()
{
    for (int i=0; i<taskNames.size(); ++i) {
        if(taskActivationVector[i]) {
            std::cout << "Reconnecting " << taskNames[i] << std::endl;
            taskRelayMap[taskNames[i]]->connect();
        }
    }
}

void OcraTaskWidget::hideTaskList()
{

    // buttonFrame->hide();
    tasksDisplayed = false;
}

void OcraTaskWidget::getTaskList()
{
    ocra_recipes::ClientCommunications clientComs;
    if( clientComs.open(0.1, false) ) {
        taskNames = clientComs.getTaskNames();
    } else {
        showUserInformation("No controller server running");
    }
}

void OcraTaskWidget::addTaskFrames(int taskIndex)
{
    // For some reason all the indexes in the button group are negative and start at -2...????
    taskIndex = abs(taskIndex)-2;
    std::cout << "Got task index " << taskIndex << std::endl;
    if ( (taskIndex < taskNames.size()) && (taskIndex >= 0) ) {
        if (!taskActivationVector[taskIndex]) {
            addTaskFrames(taskNames[taskIndex]);
            taskActivationVector[taskIndex] = true;
        } else {
            std::cout << "The " << taskNames[taskIndex] << " task frames are already added." << std::endl;
        }
    }
}

void OcraTaskWidget::addTaskFrames(const std::string& taskName)
{
    std::cout << "Adding task frames for " << taskName << std::endl;
    std::string taskFrameName = taskName + "-Frame";
    double frameTransparency = 0.3;
    std::string taskTargetName = taskName + "-Target";
    double targetTransparency = 0.6;

    sdf::SDF frameModel = getFrameSdfModel(taskFrameName, frameTransparency);
    sendModelMsgToGazebo(frameModel);

    sdf::SDF targetModel = getFrameSdfModel(taskTargetName, targetTransparency);
    sendModelMsgToGazebo(targetModel);

    std::shared_ptr<TaskConnectionRelay> relay = std::make_shared<TaskConnectionRelay>(taskName);
    taskRelayMap[taskName] = relay;
}

void OcraTaskWidget::showUserInformation(const std::string& message, int timeout)
{
    std::cout << "O.C.R.A. Message: "<< message << std::endl;
    informUserLabel->show();
    informUserLabel->setText(message.c_str());
    QTimer::singleShot(timeout, informUserLabel, SLOT(hide()));
}


void OcraTaskWidget::sendModelMsgToGazebo(const sdf::SDF& sdfModel)
{
    msgs::Factory msg;
    msg.set_sdf(sdfModel.ToString());
    this->factoryPub->Publish(msg);
}

sdf::SDF OcraTaskWidget::getFrameSdfModel(const std::string& frameName, double transparencyValue)
{
    sdf::SDF frameSDF;
    frameSDF.SetFromString(
        "<?xml version='1.0'?>\
        <sdf version='1.4'>\
        <model name='"+frameName+"'>\
            <pose>0 0 0.5 0 0 0</pose>\
            <static>false</static>\
            <link name='origin'>\
                <gravity>false</gravity>\
                <visual name='visual'>\
                    <transparency>"+std::to_string(transparencyValue)+"</transparency>\
                    <geometry>\
                          <box>\
                              <size>0.02 0.02 0.02</size>\
                          </box>\
                    </geometry>\
                    <material>\
                        <ambient>1 1 1 1</ambient>\
                        <diffuse>1 1 1 1</diffuse>\
                        <specular>0.1 0.1 0.1 1</specular>\
                        <emissive>0 0 0 0</emissive>\
                    </material>\
                </visual>\
            </link>\
            <link name='x_axis'>\
                <gravity>false</gravity>\
                <visual name='visual'>\
                    <pose>0.03 0.0 0.0 0.0 1.57 0.0</pose>\
                    <transparency>"+std::to_string(transparencyValue)+"</transparency>\
                    <geometry>\
                          <cylinder>\
                              <length>0.04</length>\
                              <radius>0.005</radius>\
                          </cylinder>\
                    </geometry>\
                    <material>\
                        <ambient>1 0 0 1</ambient>\
                        <diffuse>1 0 0 1</diffuse>\
                        <specular>0.1 0.1 0.1 1</specular>\
                        <emissive>0 0 0 0</emissive>\
                    </material>\
                </visual>\
            </link>\
            <link name='y_axis'>\
                <gravity>false</gravity>\
                <visual name='visual'>\
                    <pose>0.0 0.03 0.0 1.57 0.0 0.0</pose>\
                    <transparency>"+std::to_string(transparencyValue)+"</transparency>\
                    <geometry>\
                          <cylinder>\
                              <length>0.04</length>\
                              <radius>0.005</radius>\
                          </cylinder>\
                    </geometry>\
                    <material>\
                        <ambient>0 1 0 1</ambient>\
                        <diffuse>0 1 0 1</diffuse>\
                        <specular>0.1 0.1 0.1 1</specular>\
                        <emissive>0 0 0 0</emissive>\
                    </material>\
                </visual>\
            </link>\
            <link name='z_axis'>\
                <gravity>false</gravity>\
                <visual name='visual'>\
                    <pose>0.0 0.0 0.03 0.0 0.0 0.0</pose>\
                    <transparency>"+std::to_string(transparencyValue)+"</transparency>\
                    <geometry>\
                          <cylinder>\
                              <length>0.04</length>\
                              <radius>0.005</radius>\
                          </cylinder>\
                    </geometry>\
                    <material>\
                        <ambient>0 0 1 1</ambient>\
                        <diffuse>0 0 1 1</diffuse>\
                        <specular>0.1 0.1 0.1 1</specular>\
                        <emissive>0 0 0 0</emissive>\
                    </material>\
                </visual>\
            </link>\
            <plugin name='yarp_model_move' filename='libyarp_model_move.so'/>\
        </model>\
        </sdf>"
    );
    return frameSDF;
}
