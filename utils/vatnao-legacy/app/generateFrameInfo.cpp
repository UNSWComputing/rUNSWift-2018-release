#include "../../../robot/types/BallInfo.hpp"
#include "../../../robot/types/FieldBoundaryInfo.hpp"
#include "../../../robot/types/RansacTypes.hpp"
#include "../../../robot/types/FieldFeatureInfo.hpp"
#include "../../../robot/types/PostInfo.hpp"
#include "../../../robot/types/RobotInfo.hpp"
#include "../../../robot/perception/vision/Region.hpp"
#include "../../../robot/perception/vision/VisionDefinitions.hpp"

#include "generateFrameInfo.hpp"

#include <iostream>

VatnaoFrameInfo generateFrameInfo(Blackboard* blackboard, 
        Fovea &fovea_top, Fovea &fovea_bot){

    VatnaoFrameInfo frameInfo;
    frameInfo.topFrame = readFrom(vision, topFrame);
    frameInfo.botFrame = readFrom(vision, botFrame);
    frameInfo.regions = retrieveRegions(blackboard);
    frameInfo.balls = retrieveBalls(blackboard);
    frameInfo.fieldBoundaries = retrieveFieldBoundaries(blackboard);
    frameInfo.fieldLines = retrieveFieldLines(blackboard);
    frameInfo.fieldPoints = retrieveFieldPoints(blackboard);
    frameInfo.fieldLinePoints = retrieveFieldLinePoints(blackboard);
    frameInfo.goalPosts = retrieveGoalPosts(blackboard);
    frameInfo.robots = retrieveRobots(blackboard);

    frameInfo.regionData = readFrom(vision, regions);

    for (unsigned int i = 0; i < frameInfo.regionData.size(); i++) {
        if (frameInfo.regionData[i].isTopCamera()) {
            frameInfo.regionData[i].setFovea(fovea_top);
        }
        else {
            frameInfo.regionData[i].setFovea(fovea_bot);
        }
    }

    frameInfo.cameraToRR.pose = readFrom(motion, pose);
    frameInfo.cameraToRR.values = readFrom(motion, sensors);
    
    return frameInfo;
}

FrameRect rectFromBoundingBox(BBox bb, bool top, string label) {
    FrameRect r;
    r.topCamera = top;
    r.label = label;
    r.x = bb.a[0];
    r.y = bb.a[1];
    r.width = bb.width();
    r.height = bb.height();
    return r;
}

std::vector<FrameRect> retrieveRegions(Blackboard* blackboard){
    std::vector<FrameRect> regionRects;
    std::vector<RegionI> boxes = readFrom(vision, regions);

    for (std::vector<RegionI>::iterator it = boxes.begin(); it != boxes.end(); ++it) {
        regionRects.push_back(
                rectFromBoundingBox(
                    it->getBoundingBoxRaw(),
                    it->isTopCamera(),
                    "region"
                )
        );
    }

    return regionRects;
}

std::vector<FrameRect> retrieveBalls(Blackboard* blackboard){
    std::vector<FrameRect> ballRects;
    std::vector<BallInfo> boxes = readFrom(vision, balls);

    for (std::vector<BallInfo>::iterator it = boxes.begin(); it != boxes.end(); ++it) {
        int x = (*it).imageCoords[0] - (*it).radius; 
        int y = (*it).imageCoords[1] - (*it).radius;
        int diameter = (*it).radius * 2;

        if (!it->topCamera) {
            y -= TOP_IMAGE_ROWS;
        }

        FrameRect r;
        r.label = "ball";
        r.topCamera = it->topCamera;
        r.x = x;
        r.y = y;
        r.width = diameter;
        r.height = diameter;
        ballRects.push_back(r);
    }

    return ballRects;
}

std::vector<FrameLine> retrieveFieldBoundaries(Blackboard* blackboard){
    std::vector<FrameLine> fieldBoundaryLine;
    std::vector<FieldBoundaryInfo> fieldBoundaries = readFrom(vision, fieldBoundaries);

    for (std::vector<FieldBoundaryInfo>::iterator it = fieldBoundaries.begin(); it != fieldBoundaries.end(); ++it) {
        RANSACLine fieldBoundary = (*it).imageBoundary;

        FrameLine l;
        l.topCamera = true;
        l.label = "boundary";
        l.x1 = fieldBoundary.p1[0];
        l.y1 = fieldBoundary.p1[1];
        l.x2 = fieldBoundary.p2[0];
        l.y2 = fieldBoundary.p2[1];
        fieldBoundaryLine.push_back(l);
    }

    return fieldBoundaryLine;
}

std::vector<FrameLine> retrieveFieldLines(Blackboard* blackboard){
    std::vector<FrameLine> fieldLines;
    std::vector<FieldFeatureInfo> fieldFeatures = readFrom(vision, fieldFeatures);

    for (std::vector<FieldFeatureInfo>::iterator it = fieldFeatures.begin(); it != fieldFeatures.end(); ++it) {
        FieldFeatureInfo info = *it;

        if (info.type == FieldFeatureInfo::fLine) {
            FrameLine l;
            l.topCamera = true;
            l.label = "line";
            l.x1 = info.line.p1[0];
            l.y1 = info.line.p1[1];
            l.x2 = info.line.p2[0];
            l.y2 = info.line.p2[1];
            fieldLines.push_back(l);
        }
    }

    return fieldLines;
}

std::vector<FramePoint> retrieveFieldPoints(Blackboard* blackboard){
    std::vector<FramePoint> fieldPoints;
    std::vector<FieldFeatureInfo> fieldFeatures = readFrom(vision, fieldFeatures);

    for (std::vector<FieldFeatureInfo>::iterator it = fieldFeatures.begin(); it != fieldFeatures.end(); ++it) {
        FieldFeatureInfo info = *it;

        if (info.type == FieldFeatureInfo::fCorner) {
            FramePoint p;
            p.label = "C";
            p.topCamera = true;
            p.x = info.corner.p[0];
            p.y = info.corner.p[1];
            fieldPoints.push_back(p);
        } else if (info.type == FieldFeatureInfo::fTJunction) {
            FramePoint p;
            p.label = "T";
            p.topCamera = true;
            p.x = info.tjunction.p[0];
            p.y = info.tjunction.p[1];
            fieldPoints.push_back(p);
        } else if (info.type == FieldFeatureInfo::fPenaltySpot) {
            FramePoint p;
            p.label = "P";
            p.topCamera = true;
            p.x = info.penaltyspot.p[0];
            p.y = info.penaltyspot.p[1];
            fieldPoints.push_back(p);
        } else if (info.type == FieldFeatureInfo::fXJunction) {
            FramePoint p;
            p.label = "X";
            p.topCamera = true;
            p.x = info.xjunction.p[0];
            p.y = info.xjunction.p[1];
            fieldPoints.push_back(p);
        }
    }

    return fieldPoints;
}

std::vector<FramePoint> retrieveFieldLinePoints(Blackboard* blackboard) {
    std::vector<FramePoint> fieldLinePoints;
    std::vector<FieldFeatureInfo> fieldFeatures = readFrom(vision, fieldFeatures);

    for (std::vector<FieldFeatureInfo>::iterator it = fieldFeatures.begin(); it != fieldFeatures.end(); ++it) {
        FieldFeatureInfo info = *it;

        if (info.type == FieldFeatureInfo::fFieldLinePoint) {
            std::vector<FieldLinePointInfo> flpInfo = info.fieldlinepoints;

            for (std::vector<FieldLinePointInfo>::iterator pit = flpInfo.begin(); pit != flpInfo.end(); ++pit) {
                FieldLinePointInfo pointInfo = *pit;
                FramePoint p;
                p.label = "field line point";
                p.topCamera = true;
                p.x = pointInfo.p[0];
                p.y = pointInfo.p[1];
                fieldLinePoints.push_back(p);
            }
        }
    }

    return fieldLinePoints;
}

std::vector<FrameRect> retrieveGoalPosts(Blackboard* blackboard){
    std::vector<FrameRect> posts;
    std::vector<PostInfo> bbPosts = readFrom(vision, posts);

    for (std::vector<PostInfo>::iterator it = bbPosts.begin(); it != bbPosts.end(); ++it) {
        posts.push_back(
                rectFromBoundingBox(
                    it->imageCoords,
                    true,
                    "goal_post"
                )
        );
    }

    return posts;
}

std::vector<FrameRect> retrieveRobots(Blackboard* blackboard) {
    std::vector<FrameRect> robots;
    std::vector<RobotInfo> bbRobots = readFrom(vision, robots);

    for (std::vector<RobotInfo>::iterator it = bbRobots.begin(); it != bbRobots.end(); ++it) {
        robots.push_back(
                rectFromBoundingBox(
                    it->imageCoords,
                    true,
                    "robot"
                )
        );
    }

    return robots;
}
