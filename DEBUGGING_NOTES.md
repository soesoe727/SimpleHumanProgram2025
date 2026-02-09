# SpaceMouse 動作確認メモ

## 問題
SpaceMouseを動かしてもSetCameraMatrixが呼ばれない

## 原因の可能性
1. CNavigationModelのコンストラクタでmultiThreaded=trueを指定しているが、マルチスレッドモードでは明示的な接続処理が必要な可能性
2. Write()メソッドの呼び出しが必要だが、正しいシグネチャがわからない
3. navlibへの接続が確立されていない

## 対策
CNavigationModelをシングルスレッドモードに変更してテストする

変更前: `CNavigationModel(IViewModelNavigation *vm) : base_type(true, false), m_ivm(vm) {}`
変更後: `CNavigationModel(IViewModelNavigation *vm) : base_type(false, false), m_ivm(vm) {}`

これにより、メインスレッドでnavlibが動作し、Update()呼び出しで明示的にイベントを処理できるようになる。
