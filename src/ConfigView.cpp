#include "ConfigView.h"
#include "engine/RecordingManager.h"
#include "widgets/Painter.h"
#include "widgets/Color.h"
#include "widgets/BoxLayout.h"
#include "widgets/PushButton.h"
#include "widgets/TextureGL.h"
#include "widgets/Application.h"
#include "widgets/BitmapFontGL.h"
#include "DropDownList.h"
#include "widgets/Label.h"
#include "AudioView.h"
#include "ColorDropDownList.h"

namespace BackyardBrains {

ConfigView::ConfigView(RecordingManager &mngr, AudioView &audioView, Widget *parent) : Widget(parent), _manager(mngr), _audioView(audioView) {
	Widgets::PushButton *closeButton = new Widgets::PushButton(this);
	closeButton->clicked.connect(this, &ConfigView::closePressed);
	closeButton->setNormalTex(Widgets::TextureGL::get("data/config.png"));
	closeButton->setHoverTex(Widgets::TextureGL::get("data/confighigh.png"));

	Widgets::Label *topLabel = new Widgets::Label(this);
	topLabel->setText("Config");
	topLabel->updateSize();

	std::vector<Widgets::Color> c(AudioView::COLORS, AudioView::COLORS+AudioView::COLOR_NUM);
	std::vector<ColorDropDownList *> clrs(_manager.recordingDevices().size());
	_catchers.reserve(clrs.size());

	Widgets::Widget *group = new Widgets::Widget(this);
	group->setSizeHint(Widgets::Size(500,400));
	Widgets::BoxLayout *gvbox = new Widgets::BoxLayout(Widgets::Vertical, group);

	if(!_manager.fileMode()) {
		Widgets::BoxLayout *mutehbox = new Widgets::BoxLayout(Widgets::Horizontal);
		Widgets::Label *muteLabel = new Widgets::Label(group);
		muteLabel->setText("Mute Speakers while recording:");
		muteLabel->updateSize();

		_muteCKBox = new Widgets::PushButton(group);
		if(_manager.player().volume() == 0)
			_muteCKBox->setNormalTex(Widgets::TextureGL::get("data/ckboxon.png"));
		else
			_muteCKBox->setNormalTex(Widgets::TextureGL::get("data/ckboxoff.png"));
		_muteCKBox->setSizeHint(Widgets::Size(16,16));
		_muteCKBox->clicked.connect(this, &ConfigView::mutePressed);

		mutehbox->addWidget(muteLabel);
		mutehbox->addSpacing(10);
		mutehbox->addWidget(_muteCKBox, Widgets::AlignVCenter);
		mutehbox->addSpacing(50);
		gvbox->addLayout(mutehbox);
		gvbox->addSpacing(40);
	}

	for(unsigned int i = 0; i < _manager.recordingDevices().size(); i++) {
		if(!_manager.recordingDevices()[i].enabled)
			continue;
		clrs[i] = new ColorDropDownList(group);
		clrs[i]->setContent(c);

		_catchers.push_back(SignalCatcher(i, this));
		clrs[i]->selectionChanged.connect(&_catchers[i], &SignalCatcher::catchColor);
		Widgets::Label *name = new Widgets::Label(group);
		name->setText(_manager.recordingDevices()[i].name.c_str());
		name->updateSize();

		Widgets::BoxLayout *ghbox = new Widgets::BoxLayout(Widgets::Horizontal);
		ghbox->addWidget(clrs[i]);
		ghbox->addSpacing(20);
		ghbox->addWidget(name,Widgets::AlignVCenter);
		gvbox->addLayout(ghbox);
		gvbox->addSpacing(15);
	}

	for(int i = 0; i < audioView.channelCount(); i++)
    {
		clrs[audioView.channelVirtualDevice(i)]->setSelection(audioView.channelColor(i));
    }

    
    //Serial  config widgets
    
    Widgets::Label *name2 = new Widgets::Label(group);
    name2->setText("Select port:");
    name2->updateSize();
    gvbox->addSpacing(0);
    gvbox->addWidget(name2, Widgets::AlignLeft);
    
    
    
    //Dropdown for select port
    Widgets::BoxLayout *serialHbox = new Widgets::BoxLayout(Widgets::Horizontal);
    serialPortWidget = new DropDownList(group);
    serialPortWidget->clear();
    std::list<std::string> sps =  _manager.serailPortsList();
    std::list<std::string>::iterator it;
    for(it = sps.begin();it!=sps.end();it++)
    {
        serialPortWidget->addItem(it->c_str());
    }
    serialPortWidget->setSelection(_manager.serialPortIndex());
    _catchers.push_back(SignalCatcher(_catchers.size(), this));
    serialPortWidget->indexChanged.connect(&_catchers[_catchers.size()-1], &SignalCatcher::catchPort);
    serialPortWidget->setDisabled(_manager.serialMode());
    
    serialHbox->addWidget(serialPortWidget);
    serialHbox->addSpacing(5);

    
    
    
    
    
    //Button for connect
    std::cout<<"now serial button connect: \n";
    _connectButton = new Widgets::PushButton(group);
    _connectButton->clicked.connect(this, &ConfigView::connectPressed);
    if(_manager.serialMode())
    {
            std::cout<<"Connected button \n";
        _connectButton->setNormalTex(Widgets::TextureGL::get("data/connected.png"));
        _connectButton->setHoverTex(Widgets::TextureGL::get("data/connected.png"));
    }
    else
    {
            std::cout<<"not connected button \n";
        _connectButton->setNormalTex(Widgets::TextureGL::get("data/disconnected.png"));
        _connectButton->setHoverTex(Widgets::TextureGL::get("data/disconnected.png"));
    }
    _connectButton->setSizeHint(Widgets::Size(26,26));
    serialHbox->addWidget(_connectButton);
    serialHbox->update();
    gvbox->addSpacing(3);
    gvbox->addLayout(serialHbox);
    
    
    
    
    
    
	gvbox->update();

	Widgets::BoxLayout *vbox = new Widgets::BoxLayout(Widgets::Vertical, this);
	Widgets::BoxLayout *hbox = new Widgets::BoxLayout(Widgets::Horizontal);
	hbox->addSpacing(10);
	hbox->addWidget(closeButton);
	hbox->addSpacing(17);
	hbox->addWidget(topLabel, Widgets::AlignVCenter);
	vbox->addSpacing(10);
	vbox->addLayout(hbox);
	vbox->addSpacing(20);
	vbox->addWidget(group, Widgets::AlignCenter);
    
    
    
    

	vbox->update();

}

void ConfigView::paintEvent() {
	Widgets::Color bg = Widgets::Colors::background;
	bg.a = 250;
	Widgets::Painter::setColor(bg);
	Widgets::Painter::drawRect(rect());
	
}
    
//
// Connect/dsconnect from serial port
//
void ConfigView::connectPressed()
{
    if(_manager.serialMode())
    {
        _manager.disconnectFromSerial();
    }
    else
    {
        if(!_manager.initSerial(serialPortWidget->item(serialPortWidget->selection()).c_str()))
        {
            std::cout<<"Can't init serial port. \n";
        
        }
    }
    if(_manager.serialMode())
    {
        _connectButton->setNormalTex(Widgets::TextureGL::get("data/connected.png"));
        _connectButton->setHoverTex(Widgets::TextureGL::get("data/connected.png"));
        close();
    }
    else
    {
        _connectButton->setNormalTex(Widgets::TextureGL::get("data/disconnected.png"));
        _connectButton->setHoverTex(Widgets::TextureGL::get("data/disconnected.png"));
    }
    serialPortWidget->setDisabled(_manager.serialMode());
}

void ConfigView::closePressed() {
	close();
}

void ConfigView::mutePressed() {
	if(_manager.player().volume() == 0) {
		_muteCKBox->setNormalTex(Widgets::TextureGL::get("data/ckboxoff.png"));
		_manager.player().setVolume(100);
	} else {
		_muteCKBox->setNormalTex(Widgets::TextureGL::get("data/ckboxon.png"));
		_manager.player().setVolume(0);
	}
}

void ConfigView::serialPortChanged(int virtualDevice, int portidx)
{
    _manager.changeSerialPort(portidx);
}
    
    
void ConfigView::colorChanged(int virtualDevice, int coloridx) {
	int channel = _audioView.virtualDeviceChannel(virtualDevice);
	if(channel < 0 && coloridx != 0) {
		int nchan = _audioView.addChannel(virtualDevice);
		_audioView.setChannelColor(nchan, coloridx);
	} else if(coloridx == 0) {
		_audioView.removeChannel(virtualDevice);
	} else {
		_audioView.setChannelColor(channel, coloridx);
	}
}

}
