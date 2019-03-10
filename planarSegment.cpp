#include "planarSegment.h"
/*
created at 08/28/2018
*/
void extractHorizonStringSegments(cv::Mat &edgeMat, std::vector< std::vector<cv::Point> > &horizon_strings)
{
    int width = edgeMat.cols;
    int height = edgeMat.rows;
    for(int y=0; y < height; ++y)
    {
        for(int x=0; x < width; ++x)
        {
            bool flg=false;
            std::vector<cv::Point> stringSeg;
            cv::Point p;
            p.y = y;
            if(edgeMat.at<unsigned char>(y,x-1)==1 && edgeMat.at<unsigned char>(y,x)==0)
            {
                while(x < width)
                {
                    p.x = x;
                    stringSeg.push_back(p);
                    if(edgeMat.at<unsigned char>(y,x)==1)
                    {
                        flg = true;
                        break;
                    }
                    ++x;
                }
            }
            if(flg==true)
            {
                horizon_strings.push_back(stringSeg);
            }
        }
    }
}
/*
created at 08/28/2018
*/
void extractVerticalStringSegments(cv::Mat &edgeMat, std::vector< std::vector<cv::Point> > &vertical_strings)
{
    int width = edgeMat.cols;
    int height = edgeMat.rows;
    for(int x=0; x < width; ++x)
    {
        for(int y=1; y < height; ++y)
        {
            bool flg=false;
            std::vector<cv::Point> string;
            cv::Point p;
            p.x = x;
            if(edgeMat.at<unsigned char>(y-1,x)==1 && edgeMat.at<unsigned char>(y,x)==0)
            {
                while(y<(height-1))
                {
                    p.y = y;
                    string.push_back(p);
                    if(edgeMat.at<unsigned char>(y-1,x)==0 && edgeMat.at<unsigned char>(y,x)==1)
                    {
                        flg = true;
                        break;
                    }
                    ++y;
                }
                if(flg==true)
                    vertical_strings.push_back(string);
            }
        }
    }
}
/*
created at 08/28/2018
*/
void extractLineSegs(cv::Mat &depthMat, std::vector< std::vector<cv::Point> > &strings, float slopeThresh,
                     int nType,
                     cv::Mat &lineMat, std::vector<Line_Info> &lines)
{
    int lineCount = lines.size();
    for(int i=0; i < strings.size(); ++i)
    {
        bool bStraight = true;
        for(int j=0; j < strings[i].size()-1; ++j)
        {
            cv::Point p1 = strings[i][j];
            cv::Point p2 = strings[i][j+1];
            float s1 = depthMat.at<float>(p2.y,p2.x)-depthMat.at<float>(p1.y,p1.x);
            for(int k=j; k < strings[i].size()-1; ++k)
            {
                cv::Point p3 = strings[i][k];
                cv::Point p4 = strings[i][k+1];
                float s2 = depthMat.at<float>(p4.y,p4.x)-depthMat.at<float>(p3.y,p3.x);
                if(fabs(s2-s1) > slopeThresh)
                {
                    bStraight = false;
                    break;
                }
            }
            if(bStraight==false)
                break;
        }
        if(bStraight==false)
            continue;
        if(bStraight==true)
        {
            for(int j=0; j < strings[i].size(); ++j)
            {
                lineMat.at<unsigned short>(strings[i][j].y,strings[i][j].x) = lineCount+1;
            }
            Line_Info line;
            line.startPt = strings[i][0];
            line.nPixels = strings[i].size();
            line.nType = nType;
            line.bMerged = false;
            lines.push_back(line);
        }
        ++lineCount;
    }
}
/*
created at 08/29/2018
*/
void extractLeftDigString(cv::Mat &edgeMat, std::vector< std::vector<cv::Point> > &leftStrings)
{
    int width = edgeMat.cols;
    int height = edgeMat.rows;
    for(int ix=1; ix < width; ++ix)
    {
        int y=1;
        for(int x=ix; x < width; ++x)
        {
            if(edgeMat.at<unsigned char>(y-1,x-1)==1 && edgeMat.at<unsigned char>(y,x)==0)
            {
                bool flg = false;
                std::vector<cv::Point> string;
                while(flg==false)
                {
                   cv::Point p(x,y);
                   string.push_back(p);
                   if(x > width-1 || y > height-1)
                       break;
                   if(edgeMat.at<unsigned char>(y,x)==1)
                   {
                       flg=true;
                       break;
                   }
                   ++x;
                   ++y;
                }
                if(flg==true)
                {
                    leftStrings.push_back(string);
                }
            }
            ++y;
            if(y > (height-1))
                break;
        }
    }
    for(int iy=1; iy < height; ++iy)
    {
        int x=1;
        for(int y=iy; y < height; ++y)
        {
            if(edgeMat.at<unsigned char>(y-1,x-1)==1 && edgeMat.at<unsigned char>(y,x)==0)
            {
                bool flg = false;
                std::vector<cv::Point> string;
                while(flg==false)
                {
                    cv::Point p(x,y);
                    string.push_back(p);
                    if(x > width-1 || y > height-1)
                        break;
                    if(edgeMat.at<unsigned char>(y,x)==1)
                    {
                        flg==true;
                        break;
                    }
                    ++y;
                    ++x;
                }
                if(flg==true)
                {
                    leftStrings.push_back(string);
                }
            }
            ++x;
            if(x > (width-1))
                break;
        }
    }
}
/*
created at 08/29/2018
*/
void extractRightDigString(cv::Mat &edgeMat, std::vector< std::vector<cv::Point> > &strings)
{
    int width = edgeMat.cols;
    int height = edgeMat.rows;
    for(int ix=width-2; ix > 0; --ix)
    {
        int y=1;
        for(int x=ix; x > 0; --x)
        {
            if(edgeMat.at<unsigned char>(y-1,x+1)==1 && edgeMat.at<unsigned char>(y,x)==0)
            {
                bool flg = false;
                std::vector<cv::Point> string;
                while(flg==false)
                {
                   cv::Point p(x,y);
                   string.push_back(p);
                   if(x < 1 || y > height-1)
                       break;
                   if(edgeMat.at<unsigned char>(y,x)==1)
                   {
                       flg=true;
                       break;
                   }
                   --x;
                   ++y;
                }
                if(flg==true)
                {
                    strings.push_back(string);
                }
            }
            ++y;
            if(y > (height-1))
                break;
        }
    }
    for(int iy=1; iy < height; ++iy)
    {
        int x=width-2;
        for(int y=iy; y < height; ++y)
        {
            if(edgeMat.at<unsigned char>(y-1,x+1)==1 && edgeMat.at<unsigned char>(y,x)==0)
            {
                bool flg = false;
                std::vector<cv::Point> string;
                while(flg==false)
                {
                    cv::Point p(x,y);
                    string.push_back(p);
                    if(x < 1 || y > (height-1))
                        break;
                    if(edgeMat.at<unsigned char>(y,x)==1)
                    {
                        flg=true;
                        break;
                    }
                    ++y;
                    --x;
                }
                if(flg==true)
                {
                    strings.push_back(string);
                }
            }
            --x;
            if(x < 1)
                break;
        }
    }
}
/*
created at 08/31/2018
*/
void extractAllLineSegs(cv::Mat &depthMat, cv::Mat &edgeMat, float slopeThresh,
                        cv::Mat &horizonMat, cv::Mat &verticalMat, cv::Mat &leftMat, cv::Mat &rightMat,
                        std::vector<Line_Info> &lines)
{
    std::vector< std::vector<cv::Point> > horizon_strings;
    extractHorizonStringSegments(edgeMat, horizon_strings);
    extractLineSegs(depthMat, horizon_strings, slopeThresh, 0, horizonMat, lines);
    std::vector< std::vector<cv::Point> > vertical_strings;
    extractVerticalStringSegments(edgeMat, vertical_strings);
    extractLineSegs(depthMat, vertical_strings, slopeThresh, 1, verticalMat, lines);
    std::vector< std::vector<cv::Point> > left_strings;
    extractLeftDigString(edgeMat, left_strings);
    extractLineSegs(depthMat, left_strings, slopeThresh, 2, leftMat, lines);
    std::vector< std::vector<cv::Point> > right_strings;
    extractRightDigString(edgeMat, right_strings);
    extractLineSegs(depthMat, right_strings, slopeThresh, 3, rightMat, lines);
}
/*
created at 09/01/2018
modified at 09/04/2018
*/
void mergingLines(cv::Mat &horizonLabelMat, cv::Mat &verticalLabelMat, cv::Mat &leftLabelMat, cv::Mat &rightLabelMat,
                  int nMergeLabel, cv::Mat &labelMat, std::vector<Line_Info> &lines, int nLine)
{
    int width = horizonLabelMat.cols;
    int height = horizonLabelMat.rows;
    int nTotalLines = lines.size();
    std::vector<Line_Info> mergedLines;
    lines[nLine].bMerged = true;
    mergedLines.push_back(lines[nLine]);
    int count = 0;
    while(mergedLines.size() > count)
    {
        int x = mergedLines[count].startPt.x;
        int y = mergedLines[count].startPt.y;
        for(int i=0; i < mergedLines[count].nPixels; ++i)
        {
            if(labelMat.at<unsigned short>(y,x) == 0)
            {
                int lineLabel = horizonLabelMat.at<unsigned short>(y,x);
                if(lineLabel > (nTotalLines-1))
                    ROS_ERROR("horizon::line error");
                if(lineLabel > 0 && lines[lineLabel].bMerged==false)
                {
                    lineLabel -= 1;
                    lines[lineLabel].bMerged = true;
                    mergedLines.push_back(lines[lineLabel]);
                }
                lineLabel = verticalLabelMat.at<unsigned short>(y,x);
                if(lineLabel > (nTotalLines-1))
                    ROS_ERROR("horizon::line error");
                if(lineLabel > 0 && lines[lineLabel].bMerged==false)
                {
                    lineLabel -= 1;
                    lines[lineLabel].bMerged = true;
                    mergedLines.push_back(lines[lineLabel]);
                }
                lineLabel = leftLabelMat.at<unsigned short>(y,x);
                if(lineLabel > (nTotalLines-1))
                    ROS_ERROR("left::line error");
                if(lineLabel > 0 && lines[lineLabel].bMerged==false)
                {
                    lineLabel -= 1;
                    lines[lineLabel].bMerged = true;
                    mergedLines.push_back(lines[lineLabel]);
                }
                lineLabel = rightLabelMat.at<unsigned short>(y,x);
                if(lineLabel > nTotalLines)
                    ROS_ERROR("right::line error");
                if(lineLabel > 0 && lines[lineLabel].bMerged==false)
                {
                    lineLabel -= 1;
                    lines[lineLabel].bMerged = true;
                    mergedLines.push_back(lines[lineLabel]);
                }
                labelMat.at<unsigned short>(y,x) = nMergeLabel;
            }
            switch(mergedLines[count].nType)
            {
            case 0:
                ++x;
                break;
            case 1:
                ++y;
                break;
            case 2:
                ++x;
                ++y;
                break;
            case 3:
                --x;
                ++y;
                break;
            }
            //if(x > width-1)
                //ROS_ERROR("x:error");
            //if(y > height-1)
                //ROS_ERROR("y:error");
            x = MAX(0,x);
            y = MAX(0,y);
            x = MIN(width-1,x);
            y = MIN(height-1,y);
        }
        ++count;
    }
}
/*
created at 08/31/2018
modified at 09/01/2018
modified at 09/04/2018
*/
int segment_depth(cv::Mat &depthMat, cv::Mat &edgeMat, float slopeThresh, cv::Mat &labelMat)
{
    std::vector<Line_Info> lines;
    int width = depthMat.cols;
    int height = depthMat.rows;
    cv::Mat horizonLabelMat(cvSize(width, height),CV_16UC1,cvScalar(0));
    cv::Mat verticalLabelMat(cvSize(width, height),CV_16UC1,cvScalar(0));
    cv::Mat leftLabelMat(cvSize(width, height),CV_16UC1,cvScalar(0));
    cv::Mat rightLabelMat(cvSize(width, height),CV_16UC1,cvScalar(0));
    extractAllLineSegs(depthMat, edgeMat, slopeThresh, horizonLabelMat, verticalLabelMat, leftLabelMat, rightLabelMat, lines);
    ROS_INFO("extracting line segments is completed.");
    int line_count = 0;
    int plane_count = 1;
    while(lines.size() > line_count)
    {
        bool bFlg = false;
        while(lines.size() > line_count)
        {
            if(lines[line_count].bMerged==false)
            {
                bFlg = true;
                break;
            }
            ++line_count;
        }
        if(bFlg==false)
            break;
        mergingLines(horizonLabelMat, verticalLabelMat, leftLabelMat, rightLabelMat, plane_count+1, labelMat, lines, line_count);
        ++plane_count;
    }
    ROS_INFO("merging line segments is completed.");
    return plane_count;
}
