# DexterousHand
## What Is It
"DexterousHand" is an application that corresponds to myoelectric prosthetic hands and keeps them level when gripping an object.

物を掴んだときの水平を保つ筋電義手に対応したアプリケーションです。"DexterousHand" は「器用な手」を意味します。

With a typical myoelectric prosthetic hand, it is difficult to perform movements that require fine manipulation, such as holding a glass horizontally so as not to spill its contents. Therefore, we will create and implement a myoelectric prosthetic hand control application that can automatically perform actions such as grasping an object to maintain balance and orientation simply by switching the function on and off.

一般的な筋電義手では、コップの中身をこぼさないように水平に持つなど、細かい操作が必要な動作を行うことは困難です。そこで、機能のON/OFFを切り替えるだけで、バランスと向きを保つために物を掴むなどの動作を自動的に行うことができる筋電義手制御アプリケーションを作成し、実装します。

## Development Methods
The myoelectric prosthetic hand to implement the application will be developed as a hand prosthesis for use by people with no more than the wrist.

本アプリケーションを実装するための筋電義手は、手首より先がない人が使用する手指の義手として開発します。
  
## Characeristics
現在、AIを使った筋電義手が多く開発されており、操作性の向上が進められているが、筋電位を解析するのに時間がかかり、生身と比べて動作に体感できる遅延が発生する。そのため、コップをこぼさないように持つなど、反応が遅れると困る動作や、使用者が細かいフィードバックを行わなければならない動作が難しい現状がある。本アプリケーションは、それらの動作を使用者の入力を待つことなく、システムが自動で感知、フィードバックを行うことで即応性を向上させる。

また、使用者は、手の向きを保つ動作に関して、直接筋電位の信号を使って細かい操作をする必要がないことから、短期間の訓練で使用できるようになると考えられる。

行える動作の種類に関しても、既存の筋電義手は手首の関節が動かせない、動かせても大雑把な動きになるため、この筋電義手を使用することで、多くの種類の動作を行えるようになることが期待できる。

このアプリケーションは筋電位を筋電義手の動作量の制御ではなく、機能の切り替えに使用しているため、取得する信号は高い精度でなくても良い。そのため、筋電義手を使用者に合わせて調節するための、専門的な知識を持った人材がいなくても使い始めることができる。

複雑な調節をせずに動作に必要な筋電位信号を得るため、プログラムに工夫をする。センサーからの信号にノイズが含まれることや、センサー位置の変動によって信号の値域が頻繁に変わることを前提として、異常値を検出し、除外する機能を実装する。具体的には、一つのデータを得るために数十回信号を読み込み、それらの平均値から一定以上外れたものを異常値とみなす。

## Usage Scenes

訓練が短期間で済むという長所から、筋電義手を使用したいが訓練を行っている医療機関が近くにない人に適している。特に日本では筋電義手の訓練を行っている医療機関は36か所しかないため、長期間遠方に通院するのは大きな負担となる。

また、日常生活においては

- 食事で食器を持つ
- カメラを持つ
- スマートフォンを持って見る

といった手の向きや角度を維持する必要がある状況で、角度維持の機能が使用されることで役に立つことができると考えられる。













