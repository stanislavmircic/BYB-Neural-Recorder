#ifndef BACKYARDBRAINS_CONFIGVIEW_H
#define BACKYARDBRAINS_CONFIGVIEW_H

#include "widgets/Widget.h"
#include "widgets/PushButton.h"
#include "DropDownList.h"

namespace BackyardBrains {

class RecordingManager;
class AudioView;

class ConfigView : public Widgets::Widget {
public:
	ConfigView(RecordingManager &mngr, AudioView &audioView, Widget *parent = NULL);
private:
	class SignalCatcher : public sigslot::has_slots<> {
	public:
		SignalCatcher(int virtualDevice, ConfigView *parent) : _virtualDevice(virtualDevice), _parent(parent) {}
		void catchColor(int coloridx) {
			_parent->colorChanged(_virtualDevice, coloridx);
		}
        void catchPort(int portidx) {
            _parent->serialPortChanged(_virtualDevice, portidx);
        }
	private:
		int _virtualDevice;
		ConfigView *_parent;
	};

	Widgets::PushButton *_muteCKBox;

	std::vector<SignalCatcher> _catchers;

	void colorChanged(int virtualDevice, int coloridx);
    void serialPortChanged(int virtualDevice, int portidx);
	RecordingManager &_manager;
	AudioView &_audioView;
    DropDownList *serialPortWidget;
    Widgets::PushButton *_connectButton;
    
	void paintEvent();
    void connectPressed();
	void closePressed();
	void mutePressed();

};

}
#endif
