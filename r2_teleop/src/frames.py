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

menu_frame_id     = ['/r2/left_palm', '/r2/right_palm']
control_marker_id = ['/r2/left_control_frame', '/r2/right_control_frame']
setpoint_marker_id = ['/r2/left_setpoint_frame', '/r2/right_setpoint_frame']
control_frame_id  = ['/r2/left_middle_base', '/r2/right_middle_base']

finger_frame_id = [ ['/r2/left_thumb_base', 
                        '/r2/left_thumb_medial_prime', 
                        '/r2/left_thumb_medial', 
                        '/r2/left_thumb_distal',
                        '/r2/left_index_base', 
                        '/r2/left_index_proximal', 
                        '/r2/left_index_medial', 
                        '/r2/left_middle_base', 
                        '/r2/left_middle_proximal', 
                        '/r2/left_middle_medial', 
                        '/r2/left_ring_proximal', 
                        '/r2/left_little_proximal'], 
                       ['/r2/right_thumb_base', 
                        '/r2/right_thumb_medial_prime', 
                        '/r2/right_thumb_medial', 
                        '/r2/right_thumb_distal',
                        '/r2/right_index_base', 
                        '/r2/right_index_proximal', 
                        '/r2/right_index_medial', 
                        '/r2/right_middle_base', 
                        '/r2/right_middle_proximal', 
                        '/r2/right_middle_medial', 
                        '/r2/right_ring_proximal', 
                        '/r2/right_little_proximal']]
                        

waist_frame_id    = '/r2/waist_center'
backpack_frame_id = '/r2/backpack'

neck_frame_id = ['/r2/neck_lower_pitch', '/r2/neck_roll', '/r2/neck_upper_pitch']
head_frame_id = neck_frame_id[1]

gaze_frame_id = '/r2/gaze_control_link'

posture_frame_id = ['/r2/left_shoulder_pitch', '/r2/right_shoulder_pitch']

base_frame_id = "/r2/robot_base"

