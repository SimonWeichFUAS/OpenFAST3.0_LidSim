!**********************************************************************************************************************************
!
! ElastoDyn_GlobalData.f90
! This module defines a global array for storing certain parameters
!
!**********************************************************************************************************************************

MODULE GlobalData

	IMPLICIT NONE
	REAL, DIMENSION(:,:), ALLOCATABLE 	:: dataArray 
	LOGICAL 							:: isInitialized = .FALSE.
   
	CONTAINS
   
	! Subroutine for the initialization of the global data array
	SUBROUTINE Init_GlobalData(rows)
	
		IMPLICIT NONE
		INTEGER, INTENT(IN) 			:: rows
        
        INTEGER							:: cols
		
        cols				= 920							! Specific number of columns (Channels); 
															! Inputs from ServoDyn  + Aerod. forces at every blade node + Aerod. moments at every bladde node
															! 2						+ 450       						+ 450
        
		IF (.NOT. isInitialized) THEN
			ALLOCATE(dataArray(rows, cols))
			dataArray		= 0.0
			isInitialized	= .TRUE.
		END IF
		
	END SUBROUTINE Init_GlobalData
   
    END MODULE GlobalData
    !**********************************************************************************************************************************
    