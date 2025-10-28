# PineTimeSdFw

This folder contains the archive of PineTimeSd Firmware images that are
stored on a remote server for easy downloading and installation by the
OpenSeizureDetector Android App.

The files are:

  * index.json:  A machine readable index of the available firmware files.
  * <fw1>.zip:  downloadable firmware file 1
  * <fw2>.zip   downloadable firmware file 2


The structure of index.json is:

{
  "title": "<title for app to display>",
  "description": "<description for app to display>",
  "currentVersion": <index>,
  "releases": [
     { "version": "<version no. text>",
       "description": "<description of this firmware version>",
       "fname": "<filename of this firmware>"
     },
     { "version": "<version no. text>",
       "description": "<description of this firmware version>",
       "fname": "<filename of this firmware>"
     }
     ...

  ]
}