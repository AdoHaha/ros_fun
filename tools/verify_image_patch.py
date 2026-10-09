"""Check the NumPy image repair as the workshop's ubuntu user.

Run inside a candidate container with /opt/ros/jazzy/setup.bash sourced.
Creates only /tmp artifacts and a short-lived, isolated ROS context.
"""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("ROS_DOMAIN_ID", "111")
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import cv2
import bqplot
import tornado
assert np.__version__ == '1.26.4', np.__version__
print('VERSIONS', {'numpy': np.__version__, 'matplotlib': matplotlib.__version__, 'opencv': cv2.__version__, 'bqplot': bqplot.__version__, 'tornado': tornado.version}, flush=True)
figure = plt.figure()
plt.plot([0, 1, 2], [0, 1, 0])
output = Path('/tmp/ros-fun-qa.png')
figure.savefig(output)
plt.close(figure)
assert output.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
assert output.stat().st_size > 1000
image = np.zeros((8, 8, 3), dtype=np.uint8)
image[:, :, 1] = 255
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
assert gray.shape == (8, 8) and gray[0, 0] > 0
print('PASS Agg PNG render and OpenCV ndarray operation', flush=True)

import rclpy
from rclpy.context import Context
from python_qt_binding.QtCore import QObject
from python_qt_binding.QtWidgets import QApplication
app = QApplication([])
from rqt_plot.plot import Plot
from rqt_plot.data_plot import MatDataPlot
assert MatDataPlot is not None
ros_context = Context()
rclpy.init(context=ros_context)
node = rclpy.create_node('image_patch_plot_qa', context=ros_context)
class PluginContext(QObject):
    def __init__(self):
        super().__init__()
        self.node = node
        self.widgets = []
    def argv(self):
        return ['--empty', '--pause']
    def serial_number(self):
        return 1
    def add_widget(self, widget):
        self.widgets.append(widget)
context = PluginContext()
plugin = None
try:
    plugin = Plot(context)
    assert len(context.widgets) == 1
    plugin._data_plot._switch_data_plot_widget(1)
    assert isinstance(plugin._data_plot._data_plot_widget, MatDataPlot)
    plugin._data_plot.add_curve('qa', 'QA', [0., 1., 2.], [0., 1., 0.])
    plugin._data_plot.redraw()
    app.processEvents()
    print('PASS rqt_plot full plugin + Matplotlib Qt backend instantiated and rendered offscreen', flush=True)
finally:
    if plugin is not None:
        plugin.shutdown_plugin()
    for widget in context.widgets:
        widget.close()
    node.destroy_node()
    ros_context.shutdown()
    app.processEvents()
print('ALL IMAGE PATCH QA PASSED', flush=True)
