import numpy as np
from xml.etree.ElementTree import ElementTree

class tracklet_t(object):

    object_type = None
    size, first_frame, trans, rot, state, occ, trun, amt_occ, amt_border, num_frames = \
        None, None, None, None, None, None, None, None, None, None

    def __init__(self):
        self.size = np.nan*np.ones(3, dtype=float)

    def __str__(self):
        pass

def parse_xml(fname):

    etree = ElementTree()
    with open(fname) as f:
        etree.parse(f)

    trackets_elem = etree.find('tracklets')
    tracklets = []
    tracklet_idx = 0
    num_tracklets = None

    for e in trackets_elem:

        if e.tag == 'count':
            num_tracklets = int(e.text)
        elif e.tag == 'item_version':
            pass
        elif e.tag == 'item':
            t = tracklet_t()
            is_finished = False
            has_amt = False
            frame_idx = None

            for info in e:
                if is_finished:
                    raise ValueError('Mode info on element after finished!')
                if info.tag == 'objectType':
                    t.object_type = info.text
                elif info.tag == 'h':
                    t.size[0] = float(info.text)
                elif info.tag == 'w':
                    t.size[1] = float(info.text)
                elif info.tag == 'l':
                    t.size[2] = float(info.text)
                elif info.tag == 'first_frame':
                    t.first_frame = int(info.text)
                elif info.tag == 'poses':
                    for p in info:
                        if p.tag == 'count':
                            if t.num_frames is not None:
                                raise ValueError('there are several pose lists for a single track!')
                            elif frame_idx is not None:
                                raise ValueError('?!')

                            t.num_frames = int(p.text)
                            nf = t.num_frames
                            t.trans = np.nan*np.ones((nf, 3))
                            t.rot = np.nan*np.ones((nf, 3))
                            t.state = np.nan*np.ones(nf, dtype='uint8')
                            t.occ = np.nan*np.ones((nf, 2), dtype='uint8')
                            t.trunc = np.nan*np.ones(nf, dtype='uint8')
                            t.amt_occ = np.nan*np.ones((nf, 2), dtype=float)
                            t.amt_border = np.nan*np.ones((nf, 3), dtype=float)
                            frame_idx = 0

                        elif p.tag == 'item_version':
                            pass
                        elif p.tag == 'item':
                            if frame_idx is None:
                                 raise ValueError('pose item came before number of poses!')

                            for pi in p:
                                if pi.tag == 'tx':
                                    t.trans[frame_idx, 0] = float(pi.text)
                                elif pi.tag == 'ty':
                                    t.trans[frame_idx, 1] = float(pi.text)
                                elif pi.tag == 'tz':
                                    t.trans[frame_idx, 2] = float(pi.text)
                                elif pi.tag == 'rx':
                                    t.rot[frame_idx, 0] = float(pi.text)
                                elif pi.tag == 'ry':
                                    t.rot[frame_idx, 1] = float(pi.text)
                                elif pi.tag == 'rz':
                                    t.rot[frame_idx, 2] = float(pi.text)
                                elif pi.tag == 'state':
                                    t.state[frame_idx] = int(pi.text)
                                elif pi.tag == 'occlusion':
                                    t.occ[frame_idx, 0] = int(pi.text)
                                elif pi.tag == 'occlusion_kf':
                                    t.occ[frame_idx, 1] = int(pi.text)
                                elif pi.tag == 'truncation':
                                    t.trunc[frame_idx] = int(pi.text)
                                elif pi.tag == 'amt_occlusion':
                                    t.amt_occ[frame_idx, 0] = float(pi.text)
                                    has_amt = True
                                elif pi.tag == 'amt_occlusion_kf':
                                    t.amt_occ[frame_idx, 1] = float(pi.text)
                                    has_amt = True
                                elif pi.tag == 'amt_border_l':
                                    t.amt_border[frame_idx, 0] = float(pi.text)
                                    has_amt = True
                                elif pi.tag == 'amt_border_r':
                                    t.amt_border[frame_idx, 1] = float(pi.text)
                                    has_amt = True
                                elif pi.tag == 'amt_border_kf':
                                    t.amt_border[frame_idx, 2] = float(pi.text)
                                    has_amt = True
                                else:
                                    raise ValueError('unexpected tag in poses ' + pi.tag)
                            frame_idx += 1
                        else:
                            raise ValueError('unexpected pose info' + p.tag)
                elif info.tag == 'finished':
                    is_finished = True
                else:
                    raise ValueError('unexpected tag in tracklets ' + info.tag)

            if not is_finished:
                print 'Tracklet %d not finished\n' %(tracklet_idx)
            if t.num_frames is None:
                print 'Tracklet %d has no frames\n' %(tracklet_idx)
            elif frame_idx != t.num_frames:
                print 'Tracklet %d has %d frames, but found %d\n' %\
                    (tracklet_idx, t.num_frames, frame_idx)
            if np.abs(t.rot[:,:2]).sum() > 1e-16:
                print 'Track contains rotation other than yaw'

            if not has_amt:
                t.amt_occ, t.amt_border = None, None

            tracklets.append(t)
            tracklet_idx += 1

        else:
            raise ValueError('unexpected tracklet info')

    print 'Loaded %d tracklets' % tracklet_idx

    if tracklet_idx != num_tracklets:
        print 'File has %d tracklets, found %d' %(num_tracklets, tracklet_idx)

    return tracklets