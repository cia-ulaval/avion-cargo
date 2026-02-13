# from domain.camera import LastestFrameBuffer
# from infrastructure.vision.processor.aruco_axis_adding_processor import ArucoAxisAddingProcessor
# from infrastructure.vision.threaded_pipeline import ThreadedPipeline
# from infrastructure.communication.webrtc_content_diffuser import WebRTCContentDiffuser, WebRTCConfig
#
# from infrastructure.camera.opencv_capture_adapter import OpenCVCamera
#
# buf = LastestFrameBuffer()
# camera = OpenCVCamera()
# frame_processor = ArucoAxisAddingProcessor()
# pipeline = ThreadedPipeline(camera=camera, frame_processor=frame_processor, frame_buffer=buf)
# pipeline.start()
#
# webrtc = WebRTCContentDiffuser(buf, WebRTCConfig(port=8080, stream_fps=30))
# webrtc.diffuse_video()   # bloque, ctrl-c pour arrêter
#
# pipeline.stop()
