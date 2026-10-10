"""Finish known-gallery observations from measured evidence, not a redundant timer."""
import math

def inspection_complete(outcomes,required_views,run_id,lap_id):
    """A finished route is successful only with every required confirmed view."""
    return (len(outcomes)==len(required_views) and
            {o.get('view') for o in outcomes}==set(required_views) and
            all(o.get('confirmed') is True and
                (o.get('run_id'),o.get('lap_id'))==(run_id,lap_id) for o in outcomes))

def recognition_complete(counts,plates,expected_people,expected_plates):
    """Compare camera-derived totals with the supplied scene's target counts.

    Expected counts are a task prior, never object identities or OCR answers.
    """
    values=[counts.get(k) for k in ('residents','visitors','total')]
    return (all(type(v) is int and v>=0 for v in values) and
            values[0]+values[1]==values[2]==expected_people and
            len(plates)==len(set(plates))==expected_plates)

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
