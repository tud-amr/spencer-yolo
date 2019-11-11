/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "math.h"

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_tracking_msgs/CompositeDetectedPersons.h>

#include <algorithm>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace spencer_tracking_msgs;

ros::Publisher g_filteredTracksPublisher;

float visual_confirmation_timeout;
int visual_confirmation_counter;

typedef std::map<std::string, int> MatchesPerModalityMap;
typedef uint64_t track_id;

MatchesPerModalityMap g_minMatchesPerModality;
std::map<track_id, MatchesPerModalityMap > g_actualMatchesPerTrackAndModality;
std::map<track_id, ros::Time> g_trackLastSeenAt;
// std::map<track_id, ros::Time> g_trackLastVisualConfirmation;
std::set<track_id> g_confirmedTracks;

/// Filters out any tracks which have not been confirmed visually, by also subscribing to a CompositeDetectedPersons topic
void newTrackedPersonsAndCompositesReceived(const TrackedPersons::ConstPtr& trackedPersons, const CompositeDetectedPersons::ConstPtr& compositeDetectedPersons) {
    TrackedPersons::Ptr filteredTracks(new TrackedPersons);
    filteredTracks->header = trackedPersons->header;

    std::map<track_id, const CompositeDetectedPerson*> compositeLookup;
    foreach(const CompositeDetectedPerson& composite, compositeDetectedPersons->elements) {
        ROS_ASSERT(compositeLookup.find(composite.composite_detection_id) == compositeLookup.end()); // check that composite IDs are unique
        compositeLookup[composite.composite_detection_id] = &composite;
    }

    ros::Time currentTime = ros::Time::now();
    foreach(const TrackedPerson& trackedPerson, trackedPersons->tracks) {
        track_id trackId = trackedPerson.track_id;

        // Is this track not yet confirmed?
        if(g_confirmedTracks.find(trackId) == g_confirmedTracks.end())
        {
            // Are we seeing this track for the first time?
            if(g_actualMatchesPerTrackAndModality.find(trackId) == g_actualMatchesPerTrackAndModality.end()) {
                MatchesPerModalityMap actualMatchesPerModality;
                 for(MatchesPerModalityMap::const_iterator modalityIt = g_minMatchesPerModality.begin(); modalityIt != g_minMatchesPerModality.end(); modalityIt++) {
                    actualMatchesPerModality[modalityIt->first] = 0;
                }
                g_actualMatchesPerTrackAndModality[trackId] = actualMatchesPerModality;
            }

            // Look up corresponding composite
            MatchesPerModalityMap& actualMatchesPerModality = g_actualMatchesPerTrackAndModality[trackId];
            std::map<track_id, const CompositeDetectedPerson*>::const_iterator compositeIt = compositeLookup.find(trackedPerson.detection_id);
            if(compositeIt != compositeLookup.end()) {
                const CompositeDetectedPerson* associatedComposite = compositeIt->second;

                // Increment visual confirmation count for each modality in this composite
                foreach(const DetectedPerson& originalDetection, associatedComposite->original_detections) {
                    if(g_minMatchesPerModality.find(originalDetection.modality) != g_minMatchesPerModality.end()) {
                        actualMatchesPerModality[originalDetection.modality]++;
                    }
                }

                // Check if all modalities have sufficient confirmations for this track
                size_t numConditionsFulfilled = 0;
                for(MatchesPerModalityMap::const_iterator modalityIt = actualMatchesPerModality.begin(); modalityIt != actualMatchesPerModality.end(); modalityIt++) {
                    MatchesPerModalityMap::const_iterator minMatchesPerModalityIt = g_minMatchesPerModality.find(modalityIt->first);
                    if(minMatchesPerModalityIt == g_minMatchesPerModality.end()) continue; // modality is not of interest to us (e.g. laser)
                    if(modalityIt->second >= minMatchesPerModalityIt->second) numConditionsFulfilled++;
                }
                if(numConditionsFulfilled == actualMatchesPerModality.size()) {
                    g_confirmedTracks.insert(trackId);

                    // If confirmed, add track to result list
                    filteredTracks->tracks.push_back(trackedPerson);
                }
            }
            g_trackLastSeenAt[trackId] = currentTime;
        }
        else {
            // check if track is still visually confirmed. If it has not been confirmed for a certain period of time, do not use it anymore
            MatchesPerModalityMap& actualMatchesPerModality = g_actualMatchesPerTrackAndModality[trackId];
            std::map<track_id, const CompositeDetectedPerson*>::const_iterator compositeIt = compositeLookup.find(trackedPerson.detection_id);
            // check if person is still in the composite detection message
            if(compositeIt != compositeLookup.end()) {
                const CompositeDetectedPerson* associatedComposite = compositeIt->second;
                visual_confirmation_counter = 0;
                // check if track is visually confirmed.
                foreach(const DetectedPerson& originalDetection, associatedComposite->original_detections) { 
                    if(g_minMatchesPerModality.find(originalDetection.modality) != g_minMatchesPerModality.end()) {
                        visual_confirmation_counter ++;
                        break;
                    } 
                } 
                if(visual_confirmation_counter > 0){
                    filteredTracks->tracks.push_back(trackedPerson);
                    g_trackLastSeenAt[trackId] = currentTime;
                }
                // publish visually no longer confirmed track, but do not update last timestamp, such that it gets deleted at some point
                else {
                    filteredTracks->tracks.push_back(trackedPerson);
                } 
            }
            // If the track is no longer confirmed by neither sensor, we still keep the track for a bit!! This way we can track a pedestrian across occlusions.
            // We keep the track for the same period of time as when it is no longer visually confirmed.
            else{
                filteredTracks->tracks.push_back(trackedPerson);
            }
        }
    }


    // Delete tracks which are no longer visually confirmed
    for(std::map<track_id, ros::Time>::const_iterator trackIt = g_trackLastSeenAt.begin(); trackIt != g_trackLastSeenAt.end(); trackIt++) {
        if(currentTime - trackIt->second > ros::Duration(visual_confirmation_timeout) || currentTime < trackIt->second) {
            track_id trackId = trackIt->first;
            g_trackLastSeenAt.erase(trackId);
            g_actualMatchesPerTrackAndModality.erase(trackId);
            g_confirmedTracks.erase(trackId);
        }
    }


    // Publish filtered tracks
    g_filteredTracksPublisher.publish(filteredTracks);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "visually_confirmed_track_filter");
    ros::NodeHandle nodeHandle("");
    ros::NodeHandle privateHandle("~");

    privateHandle.getParam("min_matches_per_modality", g_minMatchesPerModality);
    ROS_ASSERT(!g_minMatchesPerModality.empty());
    std::cout << "min matches per modality" << g_minMatchesPerModality.begin()->second << std::endl;

    visual_confirmation_timeout = 2.0; privateHandle.getParam("visual_confirmation_timeout", visual_confirmation_timeout);
    std::cout << "visual confirmation timeout " << visual_confirmation_timeout << std::endl;

    std::string inputTopic = "input_tracks";
    std::string outputTopic = "output_tracks";
    std::string compositeTopic = "composite_detected_persons";
    
    typedef message_filters::sync_policies::ExactTime<TrackedPersons, CompositeDetectedPersons> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    size_t queue_size = 1;
    message_filters::Subscriber<TrackedPersons> trackSubscriber(nodeHandle, inputTopic, queue_size);
    message_filters::Subscriber<CompositeDetectedPersons> compositeSubscriber(nodeHandle, compositeTopic, queue_size);

    Synchronizer inputSynchronizer(SyncPolicy(queue_size), trackSubscriber, compositeSubscriber);
    inputSynchronizer.registerCallback(&newTrackedPersonsAndCompositesReceived);

    g_filteredTracksPublisher = nodeHandle.advertise<TrackedPersons>(outputTopic, queue_size);

    std::stringstream ss;
    size_t counter = 0;
    for(MatchesPerModalityMap::const_iterator modalityIt = g_minMatchesPerModality.begin(); modalityIt != g_minMatchesPerModality.end(); modalityIt++) {
        ss << modalityIt->first << ": " << modalityIt->second << "x";
        if(counter < g_minMatchesPerModality.size() - 1) ss << " and ";
    }

    ROS_INFO_STREAM("Removing any visually un-confirmed tracks (at least " << ss.str() << ") from topic " << ros::names::remap(inputTopic) << " based upon composites on " << ros::names::remap(compositeTopic)
        << ", outputting to topic " << ros::names::remap(outputTopic) << "!");
    ros::spin();
}
