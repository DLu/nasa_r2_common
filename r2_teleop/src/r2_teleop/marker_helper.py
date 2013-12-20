#!/usr/bin/env python

"""
Copyright (c) 2012, General Motors, Co.
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

def makeBox( msg ):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.25
    marker.scale.y = msg.scale * 0.25
    marker.scale.z = msg.scale * 0.25
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    return marker

def makeSphere( msg, scale ):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * scale
    marker.scale.y = msg.scale * scale
    marker.scale.z = msg.scale * scale
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    return marker

def makeMesh( msg, mesh_str, p, sf=1 ):
    marker = Marker()
    marker.type = Marker.MESH_RESOURCE
    marker.scale.x = sf
    marker.scale.y = sf
    marker.scale.z = sf
    marker.color.r = 0.0
    marker.color.g = 0.5
    marker.color.b = 0.0
    marker.color.a = 0.5
    marker.pose = p
    marker.mesh_resource = mesh_str
    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = False
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def makeBoxMenu( msg ):
    control =  InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.always_visible = False
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def makeMeshControl( msg, mesh_name, p ):
    control =  InteractiveMarkerControl()
    control.always_visible = False
    control.markers.append( makeMesh(msg, mesh_name, p, 0.99) )
    msg.controls.append( control )
    return control

def makeSphereControl( msg, sphere ):
    control =  InteractiveMarkerControl()
    control.always_visible = False
    control.markers.append( makeSphere(msg, sphere) )
    msg.controls.append( control )
    return control

def makeXTransControl () :
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    return control

def makeYTransControl () :
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    return control

def makeZTransControl () :
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    return control

def makeXRotControl () :
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    return control

def makeYRotControl () :
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    return control

def makeZRotControl () :
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    return control

def sixAxis():
    return [makeXTransControl(), makeYTransControl(), makeZTransControl(),
            makeXRotControl(), makeYRotControl(), makeZRotControl()]

