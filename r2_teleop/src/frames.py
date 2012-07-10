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

menu_frame_id     = ['left_palm', 'right_palm']
control_marker_id = ['left_control_frame', 'right_control_frame']
setpoint_marker_id = ['left_setpoint_frame', 'right_setpoint_frame']
control_frame_id  = ['left_middle_base', 'right_middle_base']

finger_frame_id = [ ['left_thumb_base', 
                        'left_thumb_medial_prime', 
                        'left_thumb_medial', 
                        'left_thumb_distal',
                        'left_index_base', 
                        'left_index_proximal', 
                        'left_index_medial', 
                        'left_middle_base', 
                        'left_middle_proximal', 
                        'left_middle_medial', 
                        'left_ring_proximal', 
                        'left_little_proximal'], 
                       ['right_thumb_base', 
                        'right_thumb_medial_prime', 
                        'right_thumb_medial', 
                        'right_thumb_distal',
                        'right_index_base', 
                        'right_index_proximal', 
                        'right_index_medial', 
                        'right_middle_base', 
                        'right_middle_proximal', 
                        'right_middle_medial', 
                        'right_ring_proximal', 
                        'right_little_proximal']]
                        

waist_frame_id    = 'waist_center'
backpack_frame_id = 'backpack'

neck_frame_id = ['neck_lower_pitch', 'neck_roll', 'neck_upper_pitch']
head_frame_id = neck_frame_id[1]

gaze_frame_id = 'gaze_control_link'

posture_frame_id = ['left_shoulder_pitch', 'right_shoulder_pitch']

base_frame_id = "robot_base"

