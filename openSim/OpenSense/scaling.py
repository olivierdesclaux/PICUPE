## # !conda run -n opensim python

import opensim as osim

# Load model
model = osim.Model("calibrated_Rajagopal_2015.osim")

for body in model.getBodySet():
    print(body.getName())

#
# pelvisBody = model.getBodySet().get("pelvis")
# print(pelvisBody)
# print(pelvisBody.get("pelvis_imu"))
# # newMarker = osim.Marker("LASI", pelvisBody, osim.Vec3(1, 1, 1))
#
# model.setUseVisualizer(True)
# initState = model.initSystem()
# viz = model.getVisualizer()
#
# simbodyViz = viz.getSimbodyVisualizer()
# # print("Visualizer", model.getVisualizer())
# # print("Model has visualizer : ", model.hasVisualizer())
# # print("Model is valid : ", model.isValidSystem())
#
# # for body in model.getBodySet():
# #     print(body.getName())
#
#
# viz.show(initState)

