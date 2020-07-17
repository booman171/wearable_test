import re
from statistics import median

import max30102
import hrcalc

class MAXREFDES117():

    def get(self):
        # MAXREFDES117から値を取得
        m = max30102.MAX30102()
        red, ir = m.read_sequential(1000)
        m.shutdown()

        # 取得した値から心拍数、SpO2を計算
        hr, spo2 = [], []
        for i in range(37):
            measure = hrcalc.calc_hr_and_spo2(ir[25*i:25*i+100], red[25*i:25*i+100])
            # True のものを取り出す
            if measure[1]:
                hr.append(measure[0])
            if measure[3]:
                spo2.append(measure[2])

        # 中央値を取り出す
        hr = round(median(hr))
        spo2 = round(median(spo2))
        return hr, spo2

