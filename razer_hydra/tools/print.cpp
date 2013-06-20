/*********************************************************************
*
* This is free and unencumbered software released into the public domain.
* 
* Anyone is free to copy, modify, publish, use, compile, sell, or
* distribute this software, either in source code form or as a compiled
* binary, for any purpose, commercial or non-commercial, and by any
* means.
* 
* In jurisdictions that recognize copyright laws, the author or authors
* of this software dedicate any and all copyright interest in the
* software to the public domain. We make this dedication for the benefit
* of the public at large and to the detriment of our heirs and
* successors. We intend this dedication to be an overt act of
* relinquishment in perpetuity of all present and future rights to this
* software under copyright law.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
* 
* For more information, please refer to <http://unlicense.org/>
* 
**********************************************************************/
#include <cstdio>
#include <signal.h>
#include "razer_hydra/hydra.h"

using namespace razer_hydra;

static bool g_done = false;
void sigint_handler(int signal)
{
    g_done = true;
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    printf("usage: print DEVICE\n");
    return 1;
  }
  signal(SIGINT, sigint_handler);
  RazerHydra h;
  if (!h.init(argv[1]))
    return 1;

  printf("Finished initializing, now printing data stream:\n");

  while (!g_done)
  {
    if (h.poll(10, 100))
    {
      printf("%.3f %.3f %.3f      %.3f %.3f %.3f\n",
             h.pos[0].x(), h.pos[0].y(), h.pos[0].z(),
             h.pos[1].x(), h.pos[1].y(), h.pos[1].z());
    }
  }
  return 0;
}

