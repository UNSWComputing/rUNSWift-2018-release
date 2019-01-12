#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>

#include "soccer.hpp"

template<class T>
const T& Blackboard::read(const T *component) {
    return *component;
}

template<class T>
void Blackboard::write(T *component, const T& value) {
    *component = value;
}

/* ============================================================================
 *                     BACKWARDS-COMPATIBLE SERIALISATION
 * ============================================================================
 *             IF YOU ARE MODIFYING BLACKBOARD, PLEASE READ THIS!
 *
 * When changing the Blackboard (i.e. adding, removing, or changing the type of
 * a variable), there a few simple steps that must be taken to ensure that the
 * new Blackboard instance is backwards compatible. Otherwise, offnao (or other
 * dump readers) will not be able to interpret dumps prior to the change.
 *
 * FOR ANY CHANGES
 * 1. Increment the Boost class version (defined below):
 *     e.g.
 *             BOOST_CLASS_VERSION(Blackboard, n);
 *     Becomes:
 *             BOOST_CLASS_VERSION(Blackboard, n+1);
 *
 * ADDING A VARIABLE
 * 1. Increment the Boost class version (see above)
 * 2. Add the variable to the 'shallowSerialize' function with a version
 * conditional, e.g.
 *
 *     if (version > n)   // where n is the pre-increment version
 *     {
 *         ar & x;        // where x is the new variable
 *     }
 *
 * REMOVING A VARIABLE
 * 1. Increment the Boost class version (see above)
 * 2. Add a version conditional to the variable in the 'shallowSerialize'
 * function, e.g.
 *
 *     if (version <= n)  // where n is the pre-increment version
 *     {
 *         T tmp;
 *         ar & tmp;      // this is essentially a default/junk value
 *     }
 *
 * CHANGING THE TYPE OF A VARIABLE
 * 1. Increment the Boost class version (see above)
 * 2. Add a version conditional to the variable in the 'shallowSerialize'
 * function, e.g.
 *
 *     if (version <= n)  // where n is the pre-increment version
 *     {
 *          T1 tmp;
 *          ar & tmp;
 *          x = (T2)tmp;  // assuming T1 can be type-cast converted
 *                        // to T2 (conversion may be more complicated)
 *     }
 *     else
 *     {
 *          ar & x;
 *     }
 *
 * GENERAL COMMENT
 * The above guidelines must be followed when modifying the Blackboard.
 *
 * To reviewers: if you are reviewing code that has not followed the above
 * guidelines, please ask the author to make the appropriate changes before
 * approving the pull request.
 *
 * For more info, see the Wiki page titled 'Serialization'.
 * @see https://github.com/UNSWComputing/rUNSWift/wiki/Blackboard-Serialization
 */
BOOST_CLASS_VERSION(Blackboard, 23);

template<class Archive>
void Blackboard::shallowSerialize(Archive & ar, const unsigned int version) {

    if (version < 15) {
        throw std::runtime_error("Depricated 2011 dump file detected");
    }

    ar & gameController.team_red;
    ar & gameController.player_number;

    ar & motion.sensors;
    ar & motion.pose;
    ar & motion.com;
    ar & motion.odometry;
    ar & motion.active;

    ar & perception.behaviour;
    ar & perception.kinematics;
    ar & perception.localisation;
    ar & perception.total;
    ar & perception.vision;

    // This request was updated for version 19 but not sure if more is required
    ar & behaviour.request;

    ar & kinematics.sonarFiltered;
    ar & kinematics.parameters;
    ar & kinematics.sensorsLagged;

    if (1) {//(this->mask & ROBOT_FILTER_MASK) {
        ar & localisation.robotObstacles;
    } else {
        std::vector<RobotInfo> empty;
        ar & empty;
    }

    if (this->mask & LANDMARKS_MASK) {
        ar & vision.landmarks;
    } else {
        std::vector<Ipoint> empty;
        ar & empty;
    }

    /* Only serialise the things below if WHITEBOARD_MASK is not set.
     * We also ONLY want this to happen when we are serialising in Offnao,
     * which occurs when we save the dump. WHITEBOARD_MASK can only be set
     * in the save function in offnao.
     */
    if (!(this->mask & WHITEBOARD_MASK)){
        ar & vision.timestamp;
        ar & vision.goalArea;
        ar & vision.awayGoalProb;
        ar & vision.homeMapSize;
        ar & vision.awayMapSize;
        ar & vision.feet_boxes;
        ar & vision.balls;
        ar & vision.ballHint;
        ar & vision.posts;
        ar & vision.robots;
        ar & vision.fieldBoundaries;
        ar & vision.fieldFeatures;
        ar & vision.missedFrames;
        ar & vision.dxdy;

        if(version >= 17) {
            ar & vision.regions;
        }
    }

    ar & vision.topCameraSettings;
    ar & vision.botCameraSettings;

    if (version >= 17) ar & vision.lastSecond;

    ar & receiver.message;
    ar & receiver.data;
    ar & receiver.incapacitated;


    ar & localisation.robotPos;
    ar & localisation.allrobotPos;
    ar & localisation.ballLostCount;

    if(version >= 18) {
        ar & localisation.ballSeenCount;
    }

    ar & localisation.ballPosRR;
    ar & localisation.ballPosRRC;
    ar & localisation.ballVelRRC;
    if (version >= 16) {
        ar & localisation.ballVel;
        ar & localisation.ballPosUncertainty;
        ar & localisation.ballVelEigenvalue;
        ar & localisation.robotPosUncertainty;
        ar & localisation.robotHeadingUncertainty;
    }
    ar & localisation.ballNeckRelative;
    ar & localisation.ballPos;
    ar & localisation.teamBall;
    ar & localisation.sharedLocalisationBundle;
    ar & localisation.havePendingOutgoingSharedBundle;
    ar & localisation.havePendingIncomingSharedBundle;
}

template<class Archive>
void Blackboard::save(Archive & ar, const unsigned int version) const {
    // note, version is always the latest when saving
    OffNaoMask_t mask = this->mask;
    if ((mask & SALIENCY_MASK) && (!vision.topSaliency || !vision.botSaliency))
        mask &= (~SALIENCY_MASK);
    if ((mask & RAW_IMAGE_MASK) && (!vision.topFrame || !vision.botFrame))
        mask &= (~RAW_IMAGE_MASK);
    ar & boost::serialization::make_nvp("Mask", mask);

    if ((mask & BLACKBOARD_MASK) && (mask & SALIENCY_MASK)) {
        locks.serialization->lock();
        ((Blackboard*)this)->shallowSerialize(ar, version);
        // TODO(jayen): RLE
        ar & boost::serialization::
        make_nvp(
            "TopSaliency",
            boost::serialization::
            make_binary_object(
                vision.topSaliency,
                sizeof(Colour[IMAGE_COLS / TOP_SALIENCY_DENSITY][IMAGE_ROWS / TOP_SALIENCY_DENSITY])
            )
        );
        ar & boost::serialization::
        make_nvp(
            "BotSaliency",
            boost::serialization::
            make_binary_object(
                vision.botSaliency,
                sizeof(Colour[IMAGE_COLS / BOT_SALIENCY_DENSITY][IMAGE_ROWS / BOT_SALIENCY_DENSITY])
            )
        );
        locks.serialization->unlock();
    } else if (mask & BLACKBOARD_MASK) {
        ((Blackboard*)this)->shallowSerialize(ar, version);
    } else if (mask & SALIENCY_MASK) {
        // TODO(jayen): RLE
        ar & boost::serialization::
        make_nvp(
            "TopSaliency",
            boost::serialization::
            make_binary_object(
                vision.topSaliency,
                sizeof(Colour[IMAGE_COLS / TOP_SALIENCY_DENSITY][IMAGE_ROWS / TOP_SALIENCY_DENSITY])
            )
        );
        ar & boost::serialization::
        make_nvp(
            "BotSaliency",
            boost::serialization::
            make_binary_object(
                vision.botSaliency,
                sizeof(Colour[IMAGE_COLS / BOT_SALIENCY_DENSITY][IMAGE_ROWS / BOT_SALIENCY_DENSITY])
            )
        );
    }
    if (mask & RAW_IMAGE_MASK) {
        // TODO(jayen): zlib
        ar & boost::serialization::make_nvp(
            "Top Raw Image",
            boost::serialization:: make_binary_object((void *)vision.topFrame,
            sizeof(uint8_t[IMAGE_ROWS * IMAGE_COLS * 2]))
        );
        ar & boost::serialization::make_nvp(
            "Bot Raw Image",
            boost::serialization:: make_binary_object((void *)vision.botFrame,
            sizeof(uint8_t[IMAGE_ROWS * IMAGE_COLS * 2]))
        );
    }

    ar & localisation.robotPos;
}

template<class Archive>
void Blackboard::load(Archive & ar, const unsigned int version) {
    ar & mask;
    if (mask & BLACKBOARD_MASK)
        shallowSerialize(ar, version);
    if (mask & SALIENCY_MASK) {
        vision.topSaliency = (Colour*) new Colour[IMAGE_COLS / TOP_SALIENCY_DENSITY][IMAGE_ROWS / TOP_SALIENCY_DENSITY];
        // TODO(jayen): RLE
        ar & boost::serialization::make_binary_object(vision.topSaliency, sizeof(Colour[IMAGE_COLS / TOP_SALIENCY_DENSITY][IMAGE_ROWS / TOP_SALIENCY_DENSITY]));

        vision.botSaliency = (Colour*) new Colour[IMAGE_COLS / BOT_SALIENCY_DENSITY][IMAGE_ROWS / BOT_SALIENCY_DENSITY];
        // TODO(jayen): RLE
        ar & boost::serialization::make_binary_object(vision.botSaliency, sizeof(Colour[IMAGE_COLS / BOT_SALIENCY_DENSITY][IMAGE_ROWS / BOT_SALIENCY_DENSITY]));
    }
    if (mask & RAW_IMAGE_MASK) {
        vision.topFrame = new uint8_t[IMAGE_ROWS * IMAGE_COLS * 2];
        ar & boost::serialization::
        make_binary_object((void *)vision.topFrame, sizeof(uint8_t[IMAGE_ROWS * IMAGE_COLS * 2]));
        vision.botFrame = new uint8_t[IMAGE_ROWS * IMAGE_COLS * 2];
        ar & boost::serialization::
        make_binary_object((void *)vision.botFrame, sizeof(uint8_t[IMAGE_ROWS * IMAGE_COLS * 2]));
    }

    ar & localisation.robotPos;
}

namespace boost {
    namespace serialization {
        /* // boost docs are broken
        template<class Archive>
        inline void save_construct_data(Archive &ar, const Blackboard *t, const unsigned int file_version) {
            // save data required to construct instance
            ar << t->config;
        }
        */
        template<class Archive>
        inline void load_construct_data(Archive &ar, Blackboard *t, const unsigned int file_version) {
            // retrieve data from archive required to construct new instance
            boost::program_options::variables_map config;
            ar >> config;
            // invoke inplace constructor to initialize instance of Blackboard
            ::new(t) Blackboard(config);
        }
    }
}
