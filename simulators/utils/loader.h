#ifndef BARNES_HUT_FILE_READER_H
#define BARNES_HUT_FILE_READER_H

#include <string>
#include <vector>

#include "body.h"

namespace bh {

/**
 * Loads some bodies from the specified file.
 * @param filename file containing the bodies' data
 * @return bodies corresponding to the data in the file
 */
std::vector<Body> load_bodies(const std::string& filename);

}  // namespace bh

#endif  // BARNES_HUT_FILE_READER_H
