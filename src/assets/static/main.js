import { refs } from './scripts/dom.js';
import { start, stop } from './scripts/rtc.js';
import { highlightMode, refreshRuntimeLabels, renderDrone, renderTracking, setStreamState, updateCompass } from './scripts/ui.js';

refs.startButton.addEventListener('click', start);
refs.stopButton.addEventListener('click', stop);
window.addEventListener('beforeunload', stop);
window.setInterval(refreshRuntimeLabels, 1000);

highlightMode('UNKNOWN');
updateCompass(null);
renderTracking({});
renderDrone({});
setStreamState('stopped', 'Stopped');