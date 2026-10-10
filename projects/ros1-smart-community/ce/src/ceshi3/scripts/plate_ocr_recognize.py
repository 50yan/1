#!/usr/bin/env python3
"""Recognize already detected plate crops in a short-lived CPU process."""

import json
import os
import pathlib
import resource
import sys
import time

# Set before loading Paddle/OpenCV; the VM has two CPU cores.
for name in ['OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
             'NUMEXPR_NUM_THREADS']:
    os.environ[name] = '1'
os.environ['FLAGS_enable_pir_api'] = '0'
os.environ['FLAGS_use_mkldnn'] = '0'


def main(request_path, result_path):
    started = time.monotonic()
    import cv2
    import paddleocr
    from paddleocr.paddleocr import parse_args
    from tools.infer.predict_rec import TextRecognizer

    cv2.setNumThreads(1)
    request = json.loads(pathlib.Path(request_path).read_text(encoding='utf-8'))
    model_dir = pathlib.Path(request['rec_model_dir']).expanduser()
    for name in ['inference.pdmodel', 'inference.pdiparams']:
        if not (model_dir / name).is_file():
            raise RuntimeError('缺少已下载的 OCR 字符模型文件: ' + str(model_dir / name))
    args = parse_args(mMain=False)
    args.use_gpu = False
    args.enable_mkldnn = False
    args.ir_optim = False
    args.cpu_threads = 1
    args.rec_batch_num = 1
    args.rec_image_shape = '3,48,320'
    args.rec_model_dir = str(model_dir)
    args.rec_char_dict_path = str(
        pathlib.Path(paddleocr.__file__).parent / 'ppocr/utils/ppocr_keys_v1.txt')
    recognizer = TextRecognizer(args)
    print('OCR_REC_READY', flush=True)
    items = []
    for crop_path in request['crops']:
        image = cv2.imread(crop_path)
        if image is None:
            raise RuntimeError('无法读取车牌截图: ' + crop_path)
        values, _elapsed = recognizer([image])
        text, score = values[0]
        items.append({'raw_text': text, 'score': float(score)})
    result = {'items': items, 'elapsed_seconds': time.monotonic() - started,
              'peak_rss_mb': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0}
    pathlib.Path(result_path).write_text(json.dumps(result, ensure_ascii=False), encoding='utf-8')
    print('OCR_REC_DONE {:.2f}s peak {:.1f}MB'.format(
        result['elapsed_seconds'], result['peak_rss_mb']), flush=True)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
