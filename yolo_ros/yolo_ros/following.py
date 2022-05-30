from collections import defaultdict
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple
import numpy as np
from vision_msgs.msg import Detection2D, Detection2DArray
from geometry_msgs.msg import Pose2D

def distance(a: Pose2D, b: Pose2D) -> float:
    return np.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

TO_DETECT = {
    "person",
    "bicycle",
    "car",
    "motorbike",
    "truck",
    "knife",
    "scissors",
}

@dataclass
class DetHyp:
    pose: Pose2D
    score: float = 0

    def __post_init__(self):
        self.start_timer()

    def start_timer(self):
        self._lifetime = time.time()

    @property
    def lifetime(self):
        return time.time() - self._lifetime

class Follower:
    def __init__(self, distance_th: float, time_th: float) -> None:
        self.history: Dict[str, Dict[int, DetHyp]] = defaultdict(dict)
        self.distance_th = distance_th
        self.time_th = time_th
    
    def tag(self, det_id: str, center: Pose2D) -> Optional[str]:
        if det_id in TO_DETECT:
            return self._tag_right(det_id, center)
        else:
            return self._tag_recovery(center)


    def _tag_right(self, det_id: str, center: Pose2D) -> str:
        min_distance = np.inf
        tag_idx = None
        for idx, hypotesis in self.history[det_id].items():
            if not idx in self.used[det_id]:
                dist = distance(center, hypotesis.pose)
                if dist < min_distance:
                    min_distance = dist
                    tag_idx = idx

        if tag_idx is not None:
            too_far = min_distance > self.distance_th
            used = tag_idx in self.used
            too_much_time = False
            if tag_idx in self.history[det_id]:
                too_much_time = self.history[det_id][tag_idx].lifetime > self.time_th
            create_new = too_far or used or too_much_time
        else:
            create_new = True
            tag_idx = 0

        # TODO: dimenticare vecchie tag oppure renderle di nuovo disponibili
        if create_new:
            # New detection
            tag_idx = len(self.history[det_id])
            print("New tag", tag_idx)
            self.history[det_id][tag_idx] = DetHyp(center)


        self.history[det_id][tag_idx].pose = center
        self.history[det_id][tag_idx].start_timer()
        self.used[det_id].add(tag_idx)
        return f"{det_id}_{tag_idx}"

    def _tag_recovery(self, center: Pose2D) -> Optional[str]:
        min_distance = np.inf
        tag_idx = 0
        min_hyp_name = None
        for hyp_id in self.history.keys():
            for idx, hypotesis in self.history[hyp_id].items():
                dist = distance(center, hypotesis.pose)
                if dist < min_distance:
                    min_distance = dist
                    tag_idx = idx
                    min_hyp_name = hyp_id

        if min_hyp_name is None or min_distance > self.distance_th:
            return None

        if self.history[min_hyp_name][tag_idx].lifetime > self.time_th:
            return None

        self.history[min_hyp_name][tag_idx].pose = center
        self.used[min_hyp_name].add(tag_idx)
        return f"{min_hyp_name}_{tag_idx}"


    def follow(self, detections_msg: Detection2DArray) -> Detection2DArray:
        """Retrun a Detection2Darray with new id containing a following tag"""
        self.used: Dict[str, Set[int]] = defaultdict(set)
        detections: List[Detection2D] = []
        for det in detections_msg.detections:
            for hypo in det.results:
                tag = self.tag(hypo.id, det.bbox.center)
                if tag is not None:
                    hypo.id = tag
                    detections.append(det)
                break
        
        # print()
        # for det in detections:
        #     print(det.results[0].id)
        return Detection2DArray(detections=detections)
