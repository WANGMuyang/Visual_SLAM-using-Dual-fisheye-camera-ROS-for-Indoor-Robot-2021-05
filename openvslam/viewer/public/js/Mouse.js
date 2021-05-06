/*캔버스의 마우스 조작을 정리하는 등급으로 크게 2가지 기능이 있다.
마우스의 버튼 클릭과 포인터 이동의 검지.
마우스 휠의 회전 검지.
좌우 클릭이나 마우스 이동 등, 처리에 적합하지 않은 형식으로 발행되는 이벤트를 취급하기 쉬운 형태로 리스너에게 돌려주는 클래스. 
마우스 휠도 취급할 수 있다.
*/

/*클릭 이벤트 취득 클래스*/
//통제 장치에서 부여된 ID를 갖는 요소에 대하여 마우스의 이벤트를 감시한다.。
//청취자들에게는 버튼과 액션이 인수로 전달된다。
class MouseHandler {

    //컨스트럭터
    // 캔버스 아이디와 리스나 함수 전달
    // 리스너의 반환값에 false를 반환하면, 다시 버튼 누르기까지 액션은 일어나지 않는다.
    // 리스나 함수로 지정하는 인수: (btn, act, pos)
    // btn: 밀린 버튼, 좌클릭: 0, 우클릭: 2
    // act : 마우스 액션, 푸시:0, 이동:1, 릴리스:2, 범위 외로 이동:3
    // pos : 마우스 액션이 발생한 좌표, 틀은 2차원 목록
    // return : 이후 이벤트 발행을 저해하고 싶으면 false 반환
    constructor(domElement, listener) {
        let element = (domElement !== undefined) ? domElement : document;
        //여러 마우스의 액션을 인지하여 mouse 함수로 던진다.
        element.addEventListener('mousedown', function (e) {
            mouseHandler.mouse(e, 0);
        }, false);
        element.addEventListener('mousemove', function (e) {
            mouseHandler.mouse(e, 1);
        }, false);
        element.addEventListener('mouseup', function (e) {
            mouseHandler.mouse(e, 2);
        }, false);
        element.addEventListener('mouseout', function (e) {
            mouseHandler.mouse(e, 3);
        }, false);

        element.addEventListener('contextmenu', function (e) {
            e.preventDefault();
        }, false);

        this.listener = listener;

        //버튼은 누를 때만 판별 가능하므로 값을 유지해 둘 필요.
        this.btn = 0;
        //버튼 누르기 전의 마우스 호버 또는 버튼 누르기 후 마우스가 범위 밖으로 나와 다시 들어간 후의 버튼 개방.
        //감지되지 않도록 하는 플래그
        this.pushing = false;
        this.prevPos = [0, 0];
    }


    //何らかのマウスアクションを検出
    mouse(e, act) {

        //console.log(e.button + "," + act);

        // 타깃, 즉 캔버스 왼쪽 상단의 좌표를 취득하고 마우스의 좌표에서 줄이면
        // 캔버스 왼쪽 상단을 기준으로 하는 포인터 좌표 pos를 얻는다
        let rect = e.target.getBoundingClientRect();
        let pos = [e.clientX - rect.left, e.clientY - rect.top];

        // 버튼 누르기 감지.버튼의 종류를 보관하고, 리스너를 기동.
        // 리스너 반환값이 false가 아니면 enable을 true로.
        if (act == 0) {
            if (this.pushing == false) {
                //이미 있는 버튼이 눌려 있었을 경우, 이하의 블록은 실행되지 않는다.
                this.btn = e.button;
                let continuous = this.listener(this.btn, act, pos, [0, 0]);
                this.pushing = (continuous == false) ? false : true;
                this.prevPos = pos;
            }
        }
        //マウスホバー検知。
        //ボタン押下後なら，マウスドラッグとして検知する。
        else if (act == 1) {
            if (this.pushing == true) {
                let dp = [pos[0] - this.prevPos[0], pos[1] - this.prevPos[1]];
                let continuous = this.listener(this.btn, act, pos, dp);
                this.pushing = (continuous == false) ? false : true;
                this.prevPos = pos;
            }
        }
        //ボタンリリース検知。
        //リスナを起動し，強制的にenableをクリアする。
        else if (act == 2) {
            //押されたときのボタンと異なるボタンが離されたときには以下のブロックを実行しない。
            if (this.pushing == true && this.btn == e.button) {
                let dp = [pos[0] - this.prevPos[0], pos[1] - this.prevPos[1]];
                this.listener(this.btn, act, pos, dp);
                this.pushing = false;
                this.prevPos = pos;
            }
        }
        //マウスアウトを検知。
        //マウスアウトした場合は強制的にボタンアクションを終了する。
        else {
            if (this.pushing == true) {
                let dp = [pos[0] - this.prevPos[0], pos[1] - this.prevPos[1]];
                this.listener(this.btn, act, pos, dp);
                this.pushing = false;
                this.prevPos = pos;
            }
        }
    }

}

/*マウスホイール回転の検知クラス*/
//コンストラクタで検知対象のIDとイベントリスナを指定する。
class WheelHandler {

    //コンストラクタ
    //検知対象のIDとイベントリスナを登録する
    constructor(domElement, listener) {

        let element = (domElement !== undefined) ? domElement : document;
        //ホイールイベント名を取得する。
        //ホイールイベント名は環境によって違う。
        let wheelEvent;
        if ('onwheel' in document) {
            wheelEvent = 'wheel';
        } else if ('onmousewheel' in document) {
            wheelEvent = 'mousewheel'
        } else {
            wheelEvent = 'DOMMouseScroll'
        }
        element.addEventListener(wheelEvent, function (e) {
            wheelHandler.onWheel(e)
        }, false);
        this.listener = listener;
    }
    //ホイール回転を検知するイベント
    onWheel(event) {
        event.preventDefault();

        let rot;

        switch (event.deltaMode) {
            case 2:
                rot = event.deltaY * 0.025;
                break;
            case 1:
                // Zoom in lines
                rot = event.deltaY * 0.01;
                break;
            default:
                // undefined, 0, assume pixels
                rot = event.deltaY * 0.00025;
                break;
        }

        this.listener(rot);
    }
}
