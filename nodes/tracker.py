#!/usr/bin/env python
"""
Tracker classes must implement track( frame ) which returns a tuple (x,y,area)
if the tracker does not find an object then it will return None
"""

import numpy as np
import math
import cv2
import time

lk_params = dict( winSize = ( 30, 30 ),
                  maxLevel = 2,
                  criteria = ( cv2.TERM_CRITERIA_EPS |
                               cv2.TERM_CRITERIA_COUNT, 10, 0.03 ) )

feature_params = dict( maxCorners = 50,
                       qualityLevel = 0.3,
                       minDistance = 10,
                       blockSize = 10 )

class DummyTracker:
    def track( self, frame ):
        (y,x,n) = frame.shape
        cv2.imshow( 'Dummy', frame )
        return( 50, 50, 50 )

class LkTracker:
    def __init__(self):
        self.track_len = 10
        self.detect_interval = 5
        self.tracks = []
        self.frame_idx = 0
        self.frame = None
        self.mouseDown = False
        self.timers = {}

        self.userRect = None
#        cv2.namedWindow( 'LKTracker', cv2.cv.CV_WINDOW_NORMAL )
#        cv2.cv.SetMouseCallback( 'LKTracker', self.on_mouse, None )

        self.vis = None

    def on_mouse( self, event, x, y, flags, param ):
        if self.frame is None:
            return

        if event == cv2.cv.CV_EVENT_LBUTTONDOWN:
            self.mouseDown = True
            self.userRect = np.int32( ( ( x, y ), ( x, y ) ) )

        elif event == cv2.cv.CV_EVENT_MOUSEMOVE and self.mouseDown == True:
            xmin = min( self.userRect[0,0], self.userRect[1,0], x)
            xmax = max( self.userRect[0,0], self.userRect[1,0], x)
            ymin = min( self.userRect[0,1], self.userRect[1,1], y)
            ymax = max( self.userRect[0,1], self.userRect[1,1], y)
            self.userRect = np.int32( ( ( xmin, ymin ), ( xmax, ymax ) ) )
            
        elif event == cv2.cv.CV_EVENT_LBUTTONUP:
            self.mouseDown = False
            self.pickFeatures()
            self.initCamshift()
            self.userRect = None

    def initCamshift(self):
        x0,y0,x1,y1 = (self.userRect[0,0],
                       self.userRect[0,1],
                       self.userRect[1,0],
                       self.userRect[1,1])

        hsv_roi = self.hsv[ y0:y1, x0:x1 ]
        mask_roi = self.hsv_mask[ y0:y1, x0:x1 ]

        hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0,180] )
        cv2.normalize( hist, hist, 0, 255, cv2.NORM_MINMAX )
        self.hist = hist.reshape(-1)
        self.track_window = ( x0, y0, x1-x0, y1-y0 )
        self.showHist()

    def getCamShift(self):
        prob = cv2.calcBackProject( [self.hsv], [0], self.hist, [0,180], 1 )
        prob &= self.hsv_mask
        term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
        track_box, self.track_window = cv2.CamShift( prob, self.track_window, 
                                                     term_crit )
#        cv2.imshow( 'Back Projection', prob )
        return track_box
    
    def showHist(self):
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count*bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h), (int(180.0*i/bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
#        cv2.imshow('hist', img)
        
    def pickFeatures(self):
        mask = np.zeros_like( self.frame_gray )
#        import pdb; pdb.set_trace()

        cv2.rectangle( mask, tuple( self.userRect[0] ), tuple( self.userRect[1] ), 255, -1 )
#        cv2.imshow( 'userMask', mask )

        start = cv2.getTickCount() / cv2.getTickFrequency()
        p = cv2.goodFeaturesToTrack( self.frame_gray, mask = mask, **feature_params )
        self.timers['GoodFeatures'] = cv2.getTickCount() / cv2.getTickFrequency() - start

        if p is not None:
            self.tracks = [ [ (x,y) ] for x, y in np.float32(p).reshape(-1, 2) ]

    def equalizeHist(self, frame):
        splits = cv2.split( frame )
        equalized = map( cv2.equalizeHist, splits )
        return cv2.merge( equalized )

    def initializeFrame( self, frame ):
        self.frame = cv2.pyrDown( frame )
        self.hsv = cv2.cvtColor( self.frame, cv2.COLOR_BGR2HSV )
        self.hsv = self.equalizeHist( self.hsv )
        self.hsv_mask = cv2.inRange( self.hsv, np.array((0.0, 33.0, 33.0)),
                                     np.array((180.0,254.,254.)) )

        # dilate
        self.hsv_mask = cv2.dilate( self.hsv_mask, None, iterations = 5 )

#            self.frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        hsv_split = cv2.split( self.hsv )
        self.frame_gray = hsv_split[0]
#        cv2.imshow( 'sat', hsv_split[1] )
#        cv2.imshow( 'vol', hsv_split[2] )
        
        cv2.imshow( 'gray', self.frame_gray )
#        cv2.imshow( 'hsv_mask', self.hsv_mask )

    def filterOutliers( self, deviations ):
        pts = np.int32( [ tr[-1] for tr in self.tracks ] )
        if pts.size < 10: # 5 pts (10 bc x/y)
            return

        x_median = np.median( pts[:,0] )
        y_median = np.median( pts[:,1] )

        distances = ( np.square(pts[:,0] - x_median) +
                      np.square(pts[:,1] - y_median) )
        distance_median = np.median( distances )

        self.tracks = [ tr for (i,tr) in enumerate( self.tracks )
                        if distances[i] < distance_median *
                        np.square( deviations ) ]

    def runOpticalFlow( self ):
        if len(self.tracks) > 0:
            img0, img1 = self.prev_gray, self.frame_gray
            p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2 )
            start = cv2.getTickCount() / cv2.getTickFrequency()
            p1, st, err = cv2.calcOpticalFlowPyrLK( img0, img1, p0, None, **lk_params )
            p0r, st, err = cv2.calcOpticalFlowPyrLK( img1, img0, p1, None, **lk_params )
            self.timers['LKTrack'] = cv2.getTickCount() / cv2.getTickFrequency() - start
            
            d = abs( p0-p0r ).reshape(-1, 2).max(-1)
            good = d < 1
            new_tracks = []
            for tr, (x, y), good_flag in zip( self.tracks, p1.reshape(-1,2), good ):
                if not good_flag:
                    continue
                tr.append((x, y))
                if len(tr) > self.track_len:
                    del tr[0]
                    
                new_tracks.append(tr)
            
            self.tracks = new_tracks
                    
    def reDetect( self, use_camshift = True, expand_pixels = 1 ):
        if self.frame_idx % self.detect_interval == 0 and len(self.tracks) > 0:

            if use_camshift:
                mask_camshift = np.zeros_like( self.frame_gray )
                ellipse_camshift = self.getCamShift()
                cv2.ellipse( mask_camshift, ellipse_camshift, 255, -1 )

            mask_tracked = np.zeros_like( self.frame_gray )
            pts = np.int32( [ [tr[-1]] for tr in self.tracks ] )

            if len(pts) > 5:
                ellipse_tracked = cv2.fitEllipse( pts )
            else:
                ellipse_tracked = cv2.minAreaRect( pts )

            boundingRect = cv2.boundingRect( pts )
            # x0,y0,x1,y1 = ( boundingRect[0] - int(boundingRect[2] * 0.05),
            #                 boundingRect[1] - int(boundingRect[3] * 0.05),
            #                 boundingRect[0] + int(boundingRect[2] * 1.05),
            #                 boundingRect[1] + int(boundingRect[3] * 1.05))


            x0,y0,x1,y1 = ( boundingRect[0] - expand_pixels,
                            boundingRect[1] - expand_pixels,
                            boundingRect[0] + boundingRect[2] + expand_pixels,
                            boundingRect[1] + boundingRect[3] + expand_pixels )
            
            if len( pts ) > 2:
                cv2.rectangle( mask_tracked, (x0,y0), (x1,y1), 255, -1 )

            cv2.ellipse( mask_tracked, ellipse_tracked, 255, -1 )

#            cv2.imshow( 'mask_camshift', mask_camshift )
#            cv2.imshow( 'mask_tracked', mask_tracked )

            if use_camshift:
                mask = mask_camshift & mask_tracked
            else:
                mask = mask_tracked

            for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                cv2.circle(mask, (x,y), 5, 0, -1 )

#            cv2.imshow( 'mask', mask )
            start = cv2.getTickCount() / cv2.getTickFrequency()
            p = cv2.goodFeaturesToTrack( self.frame_gray, mask = mask, **feature_params )
            self.timers['GoodFeatures'] = cv2.getTickCount() / cv2.getTickFrequency() - start

            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    self.tracks.append([(x,y)])

    def drawAndGetTrack( self ):
        vis = self.frame.copy()
        
        if len(self.tracks) > 0:
            cv2.polylines( vis, [ np.int32(tr) for tr in self.tracks ], False, ( 0, 255, 0 ) )
            pts = np.int32( [ [tr[-1]] for tr in self.tracks ] )

            for [(x,y)] in pts:
                cv2.circle( vis, (x, y), 2, (0, 255, 0), -1 )

            if len( pts ) > 2:
                boundingRect = cv2.boundingRect( pts )
                x0,y0,x1,y1 = ( boundingRect[0], boundingRect[1],
                                boundingRect[0] + boundingRect[2],
                                boundingRect[1] + boundingRect[3] )
                area = boundingRect[3] * boundingRect[2]
                cx,cy = ( boundingRect[0] + boundingRect[2]/2,
                          boundingRect[1] + boundingRect[3]/2 )

                cv2.rectangle( vis, (x0,y0), (x1,y1), (0,255,255), 3, 8, 0 )

                cv2.circle( vis, (cx,cy), 5, (0,255,255), -1, 8, 0 )

        if self.userRect != None:
            cv2.rectangle( vis, tuple( self.userRect[0] ), tuple( self.userRect[1] ), ( 255, 255, 255 ) )

        i = 0
        total = 0
        for ( timer, time ) in self.timers.items():
            i += 1
            total += time
            cv2.putText( vis, '%s: %.1f ms' % ( timer, time * 1e3 ),
                         ( 20, i*20 ), cv2.FONT_HERSHEY_PLAIN, 1.0,
                         (255, 255, 255) )
            cv2.putText( vis, '%s: %.1f ms' % ( timer, time * 1e3 ),
                         ( 20, i*20 ), cv2.FONT_HERSHEY_PLAIN, 1.0,
                         ( 0, 0, 0), thickness = 2 )


        i += 1
        cv2.putText( vis, 'Total: %.1f ms' % ( total * 1e3 ),
                     ( 20, i*20 ), cv2.FONT_HERSHEY_PLAIN, 1.0,
                     (255, 255, 255) )
        cv2.putText( vis, 'Total: %.1f ms' % ( total * 1e3 ),
                     ( 20, i*20 ), cv2.FONT_HERSHEY_PLAIN, 1.0,
                     ( 0, 0, 0), thickness = 2 )

        self.vis = vis.copy()

#        cv2.imshow( 'LKTracker', self.vis )
#        cv2.waitKey( 1 )

        if len( self.tracks ) > 2:
            imageSize = self.frame.shape
            imageArea = imageSize[0]*imageSize[1]

            xRel = cx*100/imageSize[1]
            yRel = cy*100/imageSize[0]
            areaRel = math.sqrt( float( area ) / float( imageArea ) ) * 100
            return xRel,yRel,areaRel,cx,cy
        else:
            return None

    def get_vis( self ):
        return self.vis

    def track(self, frame):
        self.initializeFrame( frame )
        self.runOpticalFlow()
        self.filterOutliers( 3 )

        self.reDetect()
        trackData = self.drawAndGetTrack()

        self.frame_idx += 1
        self.prev_gray = self.frame_gray

        return trackData

class CascadeTracker( LkTracker ):
    def __init__( self, cascadeFn ):
        self.cascade = cv2.CascadeClassifier( cascadeFn )
        self.trackData = None
        self.cascade_interval = 30

        LkTracker.__init__( self )

    def initCamshift( self ):
        pass

    def track( self, frame ):
        self.frame = frame # cv2.pyrDown( frame )
        self.frame_gray = cv2.cvtColor( self.frame, cv2.COLOR_BGR2GRAY )
        self.frame_gray = cv2.equalizeHist( self.frame_gray )

        # do optical flow if we have it
        self.runOpticalFlow()
        self.trackData = self.drawAndGetTrack()

        # now look for the object every 5th frame
        detectedObject = self.detectObject()

        # if we don't have it redetect optical flow
        if detectedObject == None:
            self.filterOutliers( 4 )
            self.reDetect( False, 0 )

        # If we do have it, then re-select optical flow features
        else:
#            import pdb; pdb.set_trace()
            self.userRect = [ [ detectedObject[0],
                                detectedObject[1] ],
                              [ detectedObject[0] + detectedObject[2],
                                detectedObject[1] + detectedObject[3] ] ]
            self.pickFeatures()

            # Add points at the corners
            corners = [ [ ( detectedObject[0], detectedObject[1] ) ],
                        [ ( detectedObject[0] + detectedObject[2],
                            detectedObject[1] ) ],
                        [ ( detectedObject[0],
                            detectedObject[1] + detectedObject[3] ) ],
                        [ ( detectedObject[0] + detectedObject[2],
                            detectedObject[1] + detectedObject[3] ) ]
                        ]

            self.tracks.extend( corners )

            self.userRect = None
            self.trackData = self.drawAndGetTrack()

            # imageSize = frame.shape
            # imageArea = imageSize[0]*imageSize[1]

            # area = detectedObject[2] * detectedObject[3]
            # cx = detectedObject[0] + detectedObject[2]/2
            # cy = detectedObject[1] + detectedObject[3]/2

            # xRel = cx*100/imageSize[1]
            # yRel = cy*100/imageSize[0]
            # areaRel = math.sqrt( float( area ) / float( imageArea ) ) * 100
            # trackData = xRel,yRel,areaRel

        self.frame_idx += 1
        self.prev_gray = self.frame_gray

        return self.trackData
	
    def detectObject( self ):
        # don't do a cascade every frame
        if self.frame_idx % self.cascade_interval != 0:
            return None

        start = cv2.getTickCount() / cv2.getTickFrequency()
        objects = self.cascade.detectMultiScale( self.frame_gray,
                                                 scaleFactor=1.3,
                                                 minNeighbors=4,
                                                 minSize=(15, 15),
                                                 flags = cv2.cv.CV_HAAR_SCALE_IMAGE)
        self.timers['Cascade'] = cv2.getTickCount() / cv2.getTickFrequency() - start

        vis = self.frame_gray.copy()

        if len( objects ) > 0:
            for i in objects:
                cv2.rectangle( vis,
                              ( int(i[0]), int(i[1]) ),
                              ( int(i[0]+i[2]), int(i[1]+i[3]) ),
                              cv2.cv.CV_RGB(0,255,0), 3, 8, 0)

#        cv2.imshow( "objects", vis )

        # if we have only one object return that
        # or if we have no trackData, return the first object
        # if we have trackData and multiple objects, return the object
        # closest to the trackData
        if len( objects ) > 0:
            if len( objects ) == 1 or not self.trackData:
                return objects[0]
            else:
                distances_squared = ( np.square(objects[:,0]+objects[:,2]/2 -
                                                self.trackData[3]) +
                                      np.square(objects[:,1]+objects[:,2]/2 -
                                                self.trackData[4]) )
                min_index = np.argmin( distances_squared )
                return objects[min_index]
        else:
            return None

def main():
    import sys
    try: video_src = sys.argv[1]
    except: video_src = 0

    tracker = CascadeTracker(
        '/home/sameer/ros/falkor_ardrone/cascade/haarcascade_falkorlogopaper.xml' )
    cam = cv2.VideoCapture( video_src )

    while True:
        ret, frame = cam.read()
        if ret:
            trackData = tracker.track( frame )

            if trackData:
                print trackData

        ch = 0xFF & cv2.waitKey(1)
        if ch == 27:
            break

if __name__ == '__main__':
    main()



                    
                
                
                 
            
    
