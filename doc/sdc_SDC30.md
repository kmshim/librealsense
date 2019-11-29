# SDC30 Direct ToF Camera
SDC30은 DTOF camera로써 intel realsense SDK를 이용할 수 있도록 firmware를 구현하고, sdk용 class를 구현하고자 한다.
SDC30은 10bit depth와 320x240의 resolution을 지원한다.
이제 막 시작하는 단계이므로 calibarion과 같은 상세한 정보는 지속적으로 추가할 예정입니다.

# Notes
USE_XU_UNIT=0 -> UVC Extension Unit을 사용하지 않는다.
USE_XU_UNIT=1 -> UVC Extension Unit을 사용한다.
USE_SDC30_TPG=0 -> Depth image를 이용하여 단순히 디버깅하기 위함. --> 삭제할 예정임.
USE_SDC30_TPG=1 -> SDC30 카메라로 부터 실제 depth image를 받아서 처리한다.