#ifndef KWP2000_SID_h
#define KWP2000_SID_h

// SIDs and PIDs
#define SID_startDiagnosticSession 0x10
/**/#define diagnosticMode_standardDiagnosticMode 0x89
#define SID_clearFaultCodes 0x14
#define SID_readFaultCodes 0x18
/**/#define readFaultCodes_getSupportedFaultCodes 0x03
#define SID_readEcuIdentification 0x1A
/**/#define ECUIdentification_hardwareNumber 0x91
/**/#define ECUIdentification_standardIdentification 0x9B
/**/#define ECUIdentification_installationList 0x9F
#define SID_readDataByLocalIdentifier 0x21
#define SID_startRoutineByLocalIdentifier 0x31
/**/#define routineLocalIdentifier_START 0xB8

// This constant is added to the SID, to be used in positive responses.
#define POSITIVE_RESPONSE 0x40

#endif
