//
// Created by chenwu on 23-8-9.
//
#ifndef MYSLAM_ORBVOCABULARY_HPP
#define MYSLAM_ORBVOCABULARY_HPP

#include "DBoW2/DBoW2/FORB.h"
#include "DBoW2/DBoW2/TemplatedVocabulary.h"

namespace myslam {
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBVocabulary;
}

#endif // MYSLAM_ORBVOCABULARY_HPP
