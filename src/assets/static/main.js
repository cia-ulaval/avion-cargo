import { refs } from './scripts/dom.js';
import { start, stop } from './scripts/rtc.js';
import { highlightMode, refreshRuntimeLabels, renderDrone, renderTracking, setStreamState, updateCompass } from './scripts/ui.js';

refs.startButton.addEventListener('click', start);
refs.stopButton.addEventListener('click', stop);
document.getElementById('fullscreenButton').addEventListener('click', async () => {
  try {
    await refs.videoStage.requestFullscreen();
  } catch (error) {
    console.error('Fullscreen unavailable:', error);
  }
});
window.addEventListener('beforeunload', stop);
window.setInterval(refreshRuntimeLabels, 1000);

highlightMode('UNKNOWN');
updateCompass(null);
renderTracking({});
renderDrone({});
setStreamState('stopped', 'Stopped');
start();
