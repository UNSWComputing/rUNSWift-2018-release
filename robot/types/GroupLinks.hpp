#ifndef GROUP_LINKS_H_
#define GROUP_LINKS_H_

#define MAX_GROUPS 4000
#define MAX_LINKS 400

#include "utils/Logger.hpp"

// Thin wrapper for array to make usage not completely confusing.
struct GroupLinks
{
    // Allocate some initial space for a base set of groups.
    GroupLinks() : num_groups_(0),
                group_links_((int*)malloc(sizeof(int)*MAX_GROUPS*MAX_LINKS)),
                group_bools_((bool*)calloc(MAX_GROUPS*MAX_GROUPS, sizeof(bool))),
                         group_amounts_((int*)malloc(sizeof(int)*MAX_GROUPS)) {};

    // Free allocated memory.
    virtual ~GroupLinks()
    {
        free(group_links_);
        free(group_bools_);
        free(group_amounts_);
    }
    // The number of groups currently stored.
    int num_groups_;
    // The groups and the links between them.
    int* group_links_;
    // Which groups are linked to each other, for quick reference.
    bool* group_bools_;
    // The number of links from each group.
    int* group_amounts_;

    // True if there was room for the new group, false if not.
    bool newGroup()
    {
        if(num_groups_ >= MAX_GROUPS)
        {
            llog(ERROR) << "Too many groups created in Colour ROI."<< std::endl;
            return(false);
        }
        group_amounts_[num_groups_] = 1;
        group_links_[num_groups_*MAX_LINKS] = num_groups_;
        group_bools_[num_groups_*MAX_GROUPS+num_groups_] = true;
        ++num_groups_;

        return true;
    }

    int* begin(int group)
    {
        return(&group_links_[group*MAX_LINKS]);
    }

    int* end(int group)
    {
        return(&group_links_[group*MAX_LINKS+group_amounts_[group]]);
    }

    bool addLink(int group, int value)
    {
        // Check if the value already exists.
        if(group_bools_[group*MAX_GROUPS+value]){
            return false;
        }

        // Check if the limit for links has been exceeded.
        if(group_amounts_[group] >= MAX_LINKS)
        {
            llog(ERROR) << "Too many links created in Colour ROI."<< std::endl;
            return false;
        }
        else
        {
            group_links_[group*MAX_LINKS+group_amounts_[group]] = value;
            group_bools_[group*MAX_GROUPS+value] = true;
            group_amounts_[group]++;
            return true;
        }
    }

    int get(int group, int value)
    {
        return(group_links_[group*MAX_LINKS+value]);
    }

    void clearHigh(int group)
    {
        // First clear bool array.
        for(int link = 1; link < group_amounts_[group]; ++link)
        {
            group_bools_[group*MAX_GROUPS + group_links_[group*MAX_LINKS+link]] =
                                                                          false;
        }

        // Now reset group amount.
        group_amounts_[group] = 1;
    }

    int size()
    {
        return(num_groups_);
    }

    int size(int group)
    {
        return(group_amounts_[group]);
    }

    void fullReset()
    {
        // First clear bool array.
        for(int group = 0; group < num_groups_; ++group)
        {
            for(int link = 0; link < group_amounts_[group]; ++link)
            {
                group_bools_[group*MAX_GROUPS +
                                    group_links_[group*MAX_LINKS+link]] = false;
            }
        }

        // Reset the group count to reset everything else.
        num_groups_ = 0;
    }
};

#endif /* end of include guard: GROUP_LINKS_H_ */
