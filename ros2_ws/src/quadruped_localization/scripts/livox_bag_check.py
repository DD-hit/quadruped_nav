"""Read-only ROS 2 recording contract checks; never certify physical origin."""
from pathlib import Path
import json
import numpy as np
import yaml
from livox_recording import inspect_cloud,inspect_imu


def storage_id(path):
    path=Path(path)
    if path.is_dir():
        metadata=yaml.safe_load((path/'metadata.yaml').read_text())
        return metadata['rosbag2_bagfile_information']['storage_identifier']
    if path.suffix=='.db3':return 'sqlite3'
    if path.suffix=='.mcap':return 'mcap'
    raise ValueError('Expected a ROS 2 bag directory, .db3, or .mcap file')


def open_reader(path):
    import rosbag2_py
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(Path(path).resolve()),storage_id=storage_id(path)),rosbag2_py.ConverterOptions('',''))
    return reader


def check(path,profile):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from livox_ros_driver2.msg import CustomMsg
    path=Path(path).resolve();reader=open_reader(path)
    types={t.name:t.type for t in reader.get_all_topics_and_types()}
    lidar=profile['input']['lidar_topic'];imu=profile['input']['imu_topic']
    required={lidar:'livox_ros_driver2/msg/CustomMsg',imu:'sensor_msgs/msg/Imu'}
    for name,type_ in required.items():
        if types.get(name)!=type_:raise ValueError(f'Missing/wrong topic type: {name} expected {type_}')
    cloud_start=[];cloud_end=[];imu_times=[];forces=[];gyros=[];record_offsets=[]
    errors=[];last={};points=accepted=0;ignored=set()
    while reader.has_next():
        topic,data,record_ns=reader.read_next()
        if topic not in required:ignored.add(topic);continue
        try:
            if topic==lidar:
                m=deserialize_message(data,CustomMsg);row=inspect_cloud(m,profile)
                header=row['stamp_ns'];cloud_start.append(header);cloud_end.append(row['end_ns'])
                points+=row['points'];accepted+=row['accepted_tag_line_points']
            else:
                m=deserialize_message(data,Imu);header,a,w=inspect_imu(m,profile)
                imu_times.append(header);record_offsets.append((record_ns-header)/1e9)
                if header<=imu_times[0]+round(profile['validation']['stationary_window_s']*1e9):forces.append(a);gyros.append(w)
            if topic in last and header<=last[topic]:raise ValueError('Non-increasing message header timestamp')
            last[topic]=header
        except Exception as error:
            if len(errors)<20:errors.append(dict(topic=topic,record_time_ns=int(record_ns),error=str(error)))
    checks=[]
    def require(name,condition,**values):checks.append(dict(test=name,result='PASS' if bool(condition) else 'FAIL',**values))
    require('all_sensor_messages_satisfy_native_contract',not errors,errors=errors)
    require('both_streams_present',len(cloud_start)>1 and len(imu_times)>1,cloud_messages=len(cloud_start),imu_messages=len(imu_times))
    metrics={}
    if len(cloud_start)>1 and len(imu_times)>1:
        it=np.array(imu_times,dtype=np.int64);ct=np.array(cloud_start,dtype=np.int64);ce=np.array(cloud_end,dtype=np.int64)
        duration=(min(it[-1],ce[-1])-max(it[0],ct[0]))/1e9
        intervals=np.diff(it)/1e9;cloud_intervals=np.diff(ct)/1e9
        imu_hz=1/np.median(intervals) if np.median(intervals)>0 else 0
        scan_hz=1/np.median(cloud_intervals) if np.median(cloud_intervals)>0 else 0
        norms=np.linalg.norm(forces,axis=1);angular=np.linalg.norm(gyros,axis=1)
        coverage=float(np.mean((ct>=it[0])&(ce<=it[-1])))
        metrics=dict(common_duration_s=float(duration),median_imu_hz=float(imu_hz),median_scan_hz=float(scan_hz),
            imu_max_gap_s=float(intervals.max()),imu_gap_over_2_periods=int(np.count_nonzero(intervals>2/profile['input']['nominal_imu_hz'])),
            scan_imu_coverage_fraction=coverage,initial_acceleration_norm_median_m_s2=float(np.median(norms)),
            initial_acceleration_norm_std_m_s2=float(np.std(norms)),initial_gyro_max_rad_s=float(angular.max()),
            record_minus_imu_header_median_s=float(np.median(record_offsets)),
            native_point_count=int(points),accepted_line_tag_point_fraction=accepted/points if points else 0)
        v=profile['validation']
        require('enough_common_recording_duration',duration>=v['minimum_duration_s'],duration_s=float(duration))
        require('imu_cadence_and_gaps',abs(imu_hz/profile['input']['nominal_imu_hz']-1)<.2 and intervals.max()<v['max_imu_gap_s'],median_hz=float(imu_hz),max_gap_s=float(intervals.max()))
        require('scan_rate_matches_profile',abs(scan_hz/profile['input']['nominal_scan_hz']-1)<.25,median_hz=float(scan_hz))
        require('initial_acceleration_unit_and_stationary_norm',v['acceleration_norm_min_m_s2']<=np.median(norms)<=v['acceleration_norm_max_m_s2'],median_m_s2=float(np.median(norms)),configured_unit=profile['input']['acceleration_unit'])
        require('stationary_initialization_window',angular.max()<v['stationary_gyro_max_rad_s'] and np.std(norms)<v['stationary_acceleration_norm_std_m_s2'])
        require('scan_intervals_covered_by_imu',coverage>=v['minimum_scan_imu_coverage'],fraction=coverage)
        require('bag_clock_matches_header_clock',abs(np.median(record_offsets))<=v['max_record_header_offset_s'],median_offset_s=float(np.median(record_offsets)))
    fixture=(path/'FIXTURE_ONLY.json') if path.is_dir() else path.with_suffix('.fixture.json')
    return dict(result='PASS' if checks and all(c['result']=='PASS' for c in checks) else 'FAIL',
        bag_path=str(path),storage_id=storage_id(path),checks=checks,metrics=metrics,ignored_topics=sorted(ignored),
        data_origin='synthetic_protocol_fixture' if fixture.exists() else 'unverified_recording',
        clock_source_sync_status='not_determinable_from_CustomMsg_and_Imu_alone',
        real_hardware_validation_complete=False,full_lio_trajectory_quality_verified=False)
