"""Finish known-gallery observations from measured evidence, not a redundant timer."""
import math

def person_evidence_ready(feedback):
    if (not feedback.get('ready') or not feedback.get('two_angles_verified') or
            feedback.get('pending_person_ids') or not feedback.get('person_ids')):
        return False
    frames=feedback.get('sample_frames',{});angles=feedback.get('sample_angles',{})
    def item(d,k):return d.get(str(k),d.get(k))
    try:
        first,second=float(item(angles,0)),float(item(angles,1))
        return (item(frames,0)>=5 and item(frames,1)>=3 and
                math.isfinite(first) and math.isfinite(second) and
                abs(math.atan2(math.sin(second-first),math.cos(second-first)))>=.05 and
                feedback.get('quiet_s',0)>=.8)
    except (TypeError,ValueError):return False

def signal_hold_reason(fresh,colour,authorized,remaining,budget):
    if not fresh:return 'stale_camera_light'
    if colour=='red':return 'red'
    if colour=='yellow':return 'yellow'
    if colour!='green':return 'unconfirmed_colour'
    if not authorized:
        return 'insufficient_green_budget' if remaining is not None and remaining<budget else 'confirming_green_frames'
    return None
