"""One ROS 2 launch starts Gazebo, robot TF, RViz, logger and one controller."""
from datetime import datetime
import json
import os
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def create(context):
    arg=lambda k: context.launch_configurations[k]
    root=Path(get_package_share_directory('micromouse_practice'))
    mode=arg('mode')
    tuning=arg('tuning')
    if tuning not in ['tuned','original']:raise ValueError('tuning must be tuned or original')
    tune_file=str(root/'config/control_tuning.yaml') if tuning=='tuned' else {}
    modes=['wall_follow','mapping','flood','astar','reactive','scan','open_loop','point2point','waypoints','pure_pursuit','observe']
    if mode not in modes: raise ValueError('Unknown mode: '+mode)
    if arg('visualize').lower()=='true' and mode!='astar':
        raise ValueError('Step visualization is available with mode:=astar')
    maps=json.loads((root/'config/maps.json').read_text())
    name=arg('map')
    if name not in maps: raise ValueError('Unknown map: '+name)
    wall=mode in ['wall_follow','mapping','flood','astar','reactive','scan']
    cfg=maps[name]
    if mode=='astar' and 'planning' not in cfg:
        raise ValueError('A* requires planning-domain metadata; currently configured for migong')
    world=root/'worlds'/(cfg['world'] if wall else 'empty.world')
    start=cfg['start'] if wall else [0.,0.]
    env={'GAZEBO_MODEL_DATABASE_URI':'','LIBGL_ALWAYS_SOFTWARE':'1','SVGA_VGPU10':'0'}
    gz=ExecuteProcess(cmd=['gzserver','--verbose',str(world),'-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so'],additional_env=env,output='screen')
    desc=xacro.process_file(str(root/'urdf/turtlebot3_burger.urdf')).toxml()
    rsp=Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':desc,'use_sim_time':True}],output='screen')
    spawn=Node(package='gazebo_ros',executable='spawn_entity.py',arguments=['-entity','tb3_burger','-file',str(root/'urdf/model.sdf'),'-x',str(start[0]),'-y',str(start[1]),'-z','.02'],output='screen')
    params={'use_sim_time':True,'output_dir':os.path.expanduser(arg('output_dir'))}
    logger_options=[params]+([{'step':.015,'max_points':6000}] if mode=='astar' else [])
    logger=Node(package='maze_control',executable='tb3_odom_path',parameters=logger_options,output='screen')
    after=[logger]
    if mode in ['mapping','flood']:
        from micromouse_practice.slam_runtime import prepare_slam_runtime
        slam_env=prepare_slam_runtime()
        slam=Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',
                  parameters=[str(root/'config/online_slam.yaml'),{'use_sim_time':True}],
                  additional_env=slam_env,output='screen')
        recorder=Node(package='micromouse_practice',executable='map_recorder',parameters=[params],output='screen')
        # Fail closed if mapping disappears; never keep driving after a SLAM crash.
        after += [RegisterEventHandler(OnProcessExit(target_action=slam,on_exit=[EmitEvent(event=Shutdown(reason='SLAM process exited'))])),
                  RegisterEventHandler(OnProcessExit(target_action=recorder,on_exit=[EmitEvent(event=Shutdown(reason='Map recorder exited'))])),
                  slam,recorder]
    if mode=='flood':
        after.append(Node(package='micromouse_practice',executable='flood_navigation',parameters=[tune_file,params,{
            'goal_dx':float(cfg['goal'][0]-cfg['start'][0]),'goal_dy':float(cfg['goal'][1]-cfg['start'][1]),
            'timeout':float(arg('timeout')),'max_v':.5}],output='screen'))
    elif mode=='astar':
        after.append(Node(package='micromouse_practice',executable='astar_navigation',parameters=[params,{'map':name,'timeout':float(arg('timeout')),'visualize':arg('visualize').lower()=='true','visual_port':int(arg('visual_port'))}],output='screen'))
    elif wall:
        algorithm='flow' if tuning=='tuned' and arg('algorithm')=='sprint' else arg('algorithm')
        after.append(Node(package='micromouse_practice',executable='wall_follow',parameters=[tune_file,params,{'map':name,'algorithm':algorithm,'mode':'wall_follow' if mode=='mapping' else mode,'require_map':mode=='mapping','timeout':float(arg('timeout'))}],output='screen'))
    elif mode!='observe':
        options={'max_v':.2,'max_w':1.0}
        if arg('waypoints_json'): options['waypoints_json']=arg('waypoints_json')
        after.append(Node(package='maze_control',executable='tb3_'+mode,parameters=[params,options],output='screen'))
    def spawned(event,ctx):
        if event.returncode!=0: return [EmitEvent(event=Shutdown(reason='Robot spawn failed'))]
        return [TimerAction(period=float(arg('start_delay')),actions=after)]
    nodes=[gz,rsp,RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=spawned)),spawn]
    if arg('gui').lower()=='true': nodes.append(ExecuteProcess(cmd=['gzclient'],additional_env=env,output='screen'))
    rviz_config='s_shape.rviz' if wall and name=='s_shape' else 'practice.rviz'
    if mode=='astar': rviz_config='astar.rviz'
    if mode=='mapping': rviz_config='mapping.rviz'
    if mode=='flood': rviz_config='flood.rviz'
    if arg('rviz').lower()=='true': nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(root/'rviz'/rviz_config)],parameters=[{'use_sim_time':True}],additional_env=env,output='screen'))
    return nodes


def generate_launch_description():
    defaults=dict(mode='wall_follow',map='migong',algorithm='sprint',tuning='tuned',gui='true',rviz='true',visualize='false',visual_port='8765',timeout='1200',start_delay='0',waypoints_json='',output_dir=os.path.expanduser('~/formal_maze_results/')+datetime.now().strftime('%Y%m%d_%H%M%S'))
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=create)])
