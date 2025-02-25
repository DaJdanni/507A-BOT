#pragma once
#include "vex.h"
#include "v5lvgl.h"
#include <iostream>
#include "gifdec.h"

/**
 * MIT License
 * Copyright (c) 2019 Theo Lemay
 * https://github.com/theol0403/gif-pros
 */

class Gif {

public:

  /**
   * Construct the Gif class
   * @param fname  the gif filename on the SD card (prefixed with /usd/)
   * @param parent the LVGL parent object
   */
  Gif(const char* fname, lv_obj_t* parent);

  /**
   * Destructs and cleans the Gif class
   */
  ~Gif();

  /**
   * Pauses the GIF task
   */
  void pause();

  /**
   * Resumes the GIF task
   */
  void resume();

  /**
   * Deletes GIF and frees all allocated memory
   */
  void clean();


  bool isFinished();

private:

  gd_GIF* _gif = nullptr; // gif decoder object
  void* _gifmem = nullptr; // gif file loaded from SD into memory 
  uint8_t* _buffer = nullptr; // decoder frame buffer

  lv_color_t* _cbuf = nullptr; // canvas buffer
  lv_obj_t* _canvas = nullptr; // canvas object

  vex::timer       _timer;
  vex::brain::lcd  _lcd;
  vex::thread      _t1;

  bool finishedPlaying;

  /**
   * Cleans and frees all allocated memory
   */
  void _cleanup();

  /**
   * Render cycle, blocks until loop count exceeds gif loop count flag (if any)
   */
  void _render();

  /**
   * Calls _render()
   * @param arg Gif*
   */
  static void _render_task(void* arg);

};