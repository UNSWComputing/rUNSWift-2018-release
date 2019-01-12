#include <exception>

struct InvalidDumpFileError: public std::exception {
    virtual const char * what() const throw() {
        return "Provided dump file is invalid";
    }
};

struct NoRawImagesInDumpError: public InvalidDumpFileError {
    virtual const char * what() const throw() {
        return "Provided dump file has no raw images. Must use dumps collected with 'Raw Image' enabled.";
    }
};

struct InvalidDumpFileVeresion: public InvalidDumpFileError {
    virtual const char * what() const throw() {
        return "Provided dump file was probably not recorded with the same blackboard version.";
    }
};
