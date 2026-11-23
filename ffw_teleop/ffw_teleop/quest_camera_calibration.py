"""Read the attached device's Camera2 calibration; accept only uncropped square RGB."""
import re
import subprocess


def parse_camera2_dump(dump,serial):
    mapping=dict(re.findall(r'Device (\d+) maps to "([^"]+)"',dump))
    sections=list(re.finditer(r'Camera HAL device device@[^/]+/virtual/(\d+).*?static information:',dump))
    cameras={}
    for i,match in enumerate(sections):
        camera_id=match.group(1)
        if camera_id not in ('50','51'):continue
        text=dump[match.end():sections[i+1].start() if i+1<len(sections) else len(dump)]
        def numbers(name):
            found=re.search(re.escape(name)+r' \([^)]*\): \w+\[\d+\]\s*\[([^]]+)\]',text)
            return None if not found else [float(x) for x in found.group(1).split()]
        intrinsic=numbers('android.lens.intrinsicCalibration')
        array=numbers('android.sensor.info.preCorrectionActiveArraySize')
        if not intrinsic or not array or array[0:2]!=[0.,0.] or array[2]!=array[3]:continue
        index=next((index for index,value in mapping.items() if value==camera_id),None)
        if index is None:continue
        size=1280;scale=size/array[2];fx,fy,cx,cy,skew=intrinsic
        cameras[f'camera {index}, facing back']=dict(serial=serial,camera_id=camera_id,
            width=size,height=size,K=[[fx*scale,skew*scale,cx*scale],[0,fy*scale,cy*scale],[0,0,1]],
            distortion=[0.,0.,0.,0.,0.],source='attached device Camera2 intrinsicCalibration',
            note='Full square virtual passthrough stream; zero lens distortion assumed. Validate with board reprojection and stationarity residuals.',
            native_intrinsic=intrinsic,native_active_array=array)
    return cameras


def read_camera2(adb):
    serial=subprocess.check_output([adb,'get-serialno'],text=True,timeout=5).strip()
    dump=subprocess.check_output([adb,'shell','dumpsys','media.camera'],text=True,timeout=10)
    return parse_camera2_dump(dump,serial)


def for_frame(cameras,metadata):
    calibration=cameras.get(metadata.get('camera_label'))
    settings=metadata.get('settings',{})
    if not calibration:return None
    if metadata.get('width')!=calibration['width'] or metadata.get('height')!=calibration['height']:return None
    if settings.get('resizeMode')!='none':return None
    return calibration
