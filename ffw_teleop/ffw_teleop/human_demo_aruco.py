"""ArUco pixels and calibrated camera-to-board pose; never invent a world pose."""

class ArucoDetector:
    def __init__(self, dictionary='DICT_4X4_50'):
        import cv2
        self.cv2 = cv2
        if not dictionary.startswith('DICT_') or not hasattr(cv2.aruco, dictionary):
            raise ValueError('Unknown ArUco dictionary: '+dictionary)
        self.dictionary = dictionary
        self.codebook = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        self.detector = cv2.aruco.ArucoDetector(self.codebook) if hasattr(cv2.aruco, 'ArucoDetector') else None

    def detect(self, content, calibration=None, workspace=None):
        import numpy as np
        image = self.cv2.imdecode(np.frombuffer(content, dtype=np.uint8), self.cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError('Cannot decode camera JPEG')
        corners, ids, _ = (self.detector.detectMarkers(image) if self.detector else
                           self.cv2.aruco.detectMarkers(image, self.codebook))
        result = dict(dictionary=self.dictionary, width=image.shape[1], height=image.shape[0],
                    markers=[] if ids is None else [dict(id=int(i), corners=c.reshape(4, 2).tolist())
                                                   for i, c in zip(ids.flatten(), corners)],
                    coordinates='image_pixels', pose=None)

        if calibration is not None and workspace is not None and ids is not None:
            result['pose'] = self.board_pose(result, calibration, workspace)
        return result

    def board_pose(self, detection, calibration, workspace):
        import numpy as np
        if workspace.get('dictionary') != self.dictionary:
            return None
        side=workspace['marker_side_m']/2
        object_points=[];image_points=[];used=[]
        for marker in detection['markers']:
            centre=workspace['marker_centres_board_m'].get(str(marker['id']))
            if centre is None:continue
            offsets=np.array([[-side,side,0],[side,side,0],[side,-side,0],[-side,-side,0]])
            object_points.extend(np.array(centre)+offsets);image_points.extend(marker['corners']);used.append(marker['id'])
        if len(used)<2:return None
        obj=np.asarray(object_points,dtype=np.float64);img=np.asarray(image_points,dtype=np.float64)
        k=np.asarray(calibration['K'],dtype=float);dist=np.asarray(calibration['distortion'],dtype=float)
        ok,rvecs,tvecs,_=self.cv2.solvePnPGeneric(obj,img,k,dist,flags=self.cv2.SOLVEPNP_IPPE)
        candidates=[]
        if not ok:return None
        for rvec,tvec in zip(rvecs,tvecs):
            rvec,tvec=self.cv2.solvePnPRefineLM(obj,img,k,dist,rvec.copy(),tvec.copy())
            rotation,_=self.cv2.Rodrigues(rvec);translation=tvec.ravel()
            if np.any((obj@rotation.T+translation)[:,2]<=0) or rotation[:,2]@translation>=0:continue
            projected,_=self.cv2.projectPoints(obj,rvec,tvec,k,dist)
            error=float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-img)**2,axis=1))))
            transform=np.eye(4);transform[:3,:3]=rotation;transform[:3,3]=translation
            candidates.append((error,transform))
        if not candidates:return None
        error,transform=min(candidates,key=lambda item:item[0])
        if error>2.:return None
        return dict(T_camera_board=transform.tolist(),reprojection_px=error,marker_ids=used,
                    origin='ID4 centre; +X paper right +Y paper top +Z out of paper',
                    intrinsics_source=calibration['source'])
