/* Malloc implementation for multiple threads without lock contention.
   Copyright (C) 2001-2017 Free Software Foundation, Inc.
   This file is part of the GNU C Library.
   Contributed by Wolfram Gloger <wg@malloc.de>, 2001.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public License as
   published by the Free Software Foundation; either version 2.1 of the
   License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; see the file COPYING.LIB.  If
   not, see <http://www.gnu.org/licenses/>.  */

#include <stdbool.h>

#if HAVE_TUNABLES
# define TUNABLE_NAMESPACE malloc
#endif
#include <elf/dl-tunables.h>

/* Compile-time constants.  */

// heap的最小大小为32K
#define HEAP_MIN_SIZE (32 * 1024)
// heap的最大大小为64M，64位上默认
#ifndef HEAP_MAX_SIZE
# ifdef DEFAULT_MMAP_THRESHOLD_MAX
#  define HEAP_MAX_SIZE (2 * DEFAULT_MMAP_THRESHOLD_MAX)
# else
#  define HEAP_MAX_SIZE (1024 * 1024) /* must be a power of two */
# endif
#endif

/* HEAP_MIN_SIZE and HEAP_MAX_SIZE limit the size of mmap()ed heaps
   that are dynamically created for multi-threaded programs.  The
   maximum size must be a power of two, for fast determination of
   which heap belongs to a chunk.  It should be much larger than the
   mmap threshold, so that requests with a size just below that
   threshold can be fulfilled without creating too many heaps.  */

/***************************************************************************/

// 获得 arena 的 top chunk
#define top(ar_ptr) ((ar_ptr)->top)

/* A heap is a single contiguous memory region holding (coalesceable)
   malloc_chunks.  It is allocated with mmap() and always starts at an
   address aligned to HEAP_MAX_SIZE.  */

// 保存 heap 信息的数据结构，当前数据结构在一个 heap 的起始，仅对非 main_arena 有意义
// 64位上 heap 每次 mmap 都是按 64M 申请
// 有了 arena 如果想获得当前 arena 所有的 heap，只需要将当前 arena 指针按 HEAP_MAX_SIZE 向下对齐后获得 heap_info 即可
typedef struct _heap_info
{
  // 当前 heap 所属的 arena
  mstate ar_ptr; /* Arena for this heap. */
  // 前一个 heap 的指针
  struct _heap_info *prev; /* Previous heap. */
  // 当前 heap 使用的大小，字节数，已分配使用的大小
  size_t size;   /* Current size in bytes. */
  // 当前 heap 设置为可读可写的字节数，实际会大于 size
  size_t mprotect_size; /* Size in bytes that has been mprotected
                           PROT_READ|PROT_WRITE.  */
  /* Make sure the following data is properly aligned, particularly
     that sizeof (heap_info) + 2 * SIZE_SZ is a multiple of
     MALLOC_ALIGNMENT. */
  char pad[-6 * SIZE_SZ & MALLOC_ALIGN_MASK];
} heap_info;


// 作为分主分配区的第一个sub_heap，heap_info存放在sub_heap的头部，紧跟heap_info之后是该非主分配区的malloc_state实例，
// 紧跟malloc_state实例后，是sub_heap中的第一个chunk，但chunk的首地址必须按照MALLOC_ALIGNMENT对齐，
// 所以在malloc_state实例和第一个chunk之间可能有几个字节的pad，但如果sub_heap不是非主分配区的第一个sub_heap，
// 则紧跟heap_info后是第一个chunk，但sysmalloc()函数默认heap_info是按照MALLOC_ALIGNMENT对齐的，没有再做对齐的工作，
// 直接将heap_info后的内存强制转换成一个chunk
/* Get a compile-time error if the heap_info padding is not correct
   to make alignment work as expected in sYSMALLOc.  */
extern int sanity_check_heap_info_alignment[(sizeof (heap_info)
                                             + 2 * SIZE_SZ) % MALLOC_ALIGNMENT
                                            ? -1 : 1];

/* Thread specific data.  */
// 每个线程都定义一个 mstate 指针
static __thread mstate thread_arena attribute_tls_model_ie;

/* Arena free list.  free_list_lock synchronizes access to the
   free_list variable below, and the next_free and attached_threads
   members of struct malloc_state objects.  No other locks must be
   acquired after free_list_lock has been acquired.  */
// 访问 free_list 即 free arena 链时的锁, arena->next_free
__libc_lock_define_initialized (static, free_list_lock);
// 当前进程包含的 arena 数量
static size_t narenas = 1;
// 所有 free arena 的链
static mstate free_list;

/* list_lock prevents concurrent writes to the next member of struct
   malloc_state objects.

   Read access to the next member is supposed to synchronize with the
   atomic_write_barrier and the write to the next member in
   _int_new_arena.  This suffers from data races; see the FIXME
   comments in _int_new_arena and reused_arena.

   list_lock also prevents concurrent forks.  At the time list_lock is
   acquired, no arena lock must have been acquired, but it is
   permitted to acquire arena locks subsequently, while list_lock is
   acquired.  */

// 访问所有 arena 链时的锁，arena->next
__libc_lock_define_initialized (static, list_lock);

/* Already initialized? */
// ptmalloc 是否已经初始化过
int __malloc_initialized = -1;

/**************************************************************************/


/* arena_get() acquires an arena and locks the corresponding mutex.
   First, try the one last locked successfully by this thread.  (This
   is the common case and handled with a macro for speed.)  Then, loop
   once over the circularly linked list of arenas.  If no arena is
   readily available, create a new one.  In this latter case, `size'
   is just a hint as to how much memory will be required immediately
   in the new arena. */

// 根据 size 获得 arena，如果线程有已获得的 arena 直接返回该 arena，否则选择空闲或者新建或者复用，size在新建时作用
// 也即：
// 当前线程有之前在使用的 arena 直接使用该 arena，否则
// 从 free list 找一个空闲的 arena，否则
// 判断是否达到 arena 限制数量，没达到直接新建，达到了尝试复用 arena
// 下面的宏执行完后会返回当前线程要使用的 arena，同时 thread_arena 已被设置
#define arena_get(ptr, size) do { \
      // 获得当前线程之前获得过的 arena
      ptr = thread_arena;						      \
      arena_lock (ptr, size);						      \
  } while (0)

#define arena_lock(ptr, size) do {					      \
      // 当前线程已经获得过 arena，直接使用并上锁
      if (ptr && !arena_is_corrupt (ptr))				      \
        __libc_lock_lock (ptr->mutex);					      \
      else
        // 当前线程没有获得过 arena，尝试获得新的								      
        ptr = arena_get2 ((size), NULL);				      \
  } while (0)

/* find the heap and corresponding arena for a given ptr */

// 获得 ptr 所在的 heap，heap 按 HEAP_MAX_SIZE 对齐，直接获得对齐后地址即为 heap 起始
#define heap_for_ptr(ptr) \
  ((heap_info *) ((unsigned long) (ptr) & ~(HEAP_MAX_SIZE - 1)))

// 获得 ptr 所在的 arena，除了主分配区外，根据 ptr 所属的 heap 获得对应的 arena
// 是否属于主分配区可以根据size中的bit进行判断
#define arena_for_chunk(ptr) \
  (chunk_main_arena (ptr) ? &main_arena : heap_for_ptr (ptr)->ar_ptr)


/**************************************************************************/

/* atfork support.  */

/* The following three functions are called around fork from a
   multi-threaded process.  We do not use the general fork handler
   mechanism to make sure that our handlers are the last ones being
   called, so that other fork handlers can use the malloc
   subsystem.  */

// 用于注册 pthread_atfork，保证fork后ptmalloc的锁都是释放状态

// 父进程获得list_lock 和所有 arena 的锁
void
internal_function
__malloc_fork_lock_parent (void)
{
  // ptmalloc 没有初始化过
  if (__malloc_initialized < 1)
    return;

  /* We do not acquire free_list_lock here because we completely
     reconstruct free_list in __malloc_fork_unlock_child.  */
  // 获得 free list 的锁
  __libc_lock_lock (list_lock);

  // 遍历获得所有 arena 的锁
  for (mstate ar_ptr = &main_arena;; )
    {
      __libc_lock_lock (ar_ptr->mutex);
      ar_ptr = ar_ptr->next;
      if (ar_ptr == &main_arena)
        break;
    }
}

// 父进程释放 list_lock 和所有 arena 的锁
void
internal_function
__malloc_fork_unlock_parent (void)
{
  if (__malloc_initialized < 1)
    return;

  for (mstate ar_ptr = &main_arena;; )
    {
      __libc_lock_unlock (ar_ptr->mutex);
      ar_ptr = ar_ptr->next;
      if (ar_ptr == &main_arena)
        break;
    }
  __libc_lock_unlock (list_lock);
}

// 子进程释放 list_lock 和所有 arena 的锁
// 由于 fork 子进程不会再有父进程所有的线程，故之前的线程所有的 arena 全部空闲出来
void
internal_function
__malloc_fork_unlock_child (void)
{
  if (__malloc_initialized < 1)
    return;

  /* Push all arenas to the free list, except thread_arena, which is
     attached to the current thread.  */
  __libc_lock_init (free_list_lock);
  // 当前线程的 arena 不为空，将当前进程 att 到 thread_arena，仅当前进程使用，故为 1
  if (thread_arena != NULL)
    thread_arena->attached_threads = 1;
  free_list = NULL;
  // 其余 arena 均不再有线程使用，加入 free list
  for (mstate ar_ptr = &main_arena;; )
    {
      // 重新初始化 arena 的锁
      __libc_lock_init (ar_ptr->mutex);
      // 排除当前线程正在使用的 arena
      if (ar_ptr != thread_arena)
        {
	  /* This arena is no longer attached to any thread.  */
    // 没有线程再使用该 arena，重置
	  ar_ptr->attached_threads = 0;
    // 将当前 arena 加入 free list
          ar_ptr->next_free = free_list;
          free_list = ar_ptr;
        }
      // 获得下一个 arena
      ar_ptr = ar_ptr->next;
      // 已经遍历结束
      if (ar_ptr == &main_arena)
        break;
    }
  // 重新初始化 list lock
  __libc_lock_init (list_lock);
}

#if HAVE_TUNABLES
static inline int do_set_mallopt_check (int32_t value);
void
DL_TUNABLE_CALLBACK (set_mallopt_check) (void *valp)
{
  int32_t value = *(int32_t *) valp;
  do_set_mallopt_check (value);
  if (check_action != 0)
    __malloc_check_init ();
}

# define DL_TUNABLE_CALLBACK_FNDECL(__name, __type) \
static inline int do_ ## __name (__type value);				      \
void									      \
DL_TUNABLE_CALLBACK (__name) (void *valp)				      \
{									      \
  __type value = *(__type *) valp;					      \
  do_ ## __name (value);						      \
}

DL_TUNABLE_CALLBACK_FNDECL (set_mmap_threshold, size_t)
DL_TUNABLE_CALLBACK_FNDECL (set_mmaps_max, int32_t)
DL_TUNABLE_CALLBACK_FNDECL (set_top_pad, size_t)
DL_TUNABLE_CALLBACK_FNDECL (set_perturb_byte, int32_t)
DL_TUNABLE_CALLBACK_FNDECL (set_trim_threshold, size_t)
DL_TUNABLE_CALLBACK_FNDECL (set_arena_max, size_t)
DL_TUNABLE_CALLBACK_FNDECL (set_arena_test, size_t)
#else
/* Initialization routine. */
#include <string.h>
extern char **_environ;

static char *
internal_function
next_env_entry (char ***position)
{
  char **current = *position;
  char *result = NULL;

  while (*current != NULL)
    {
      if (__builtin_expect ((*current)[0] == 'M', 0)
          && (*current)[1] == 'A'
          && (*current)[2] == 'L'
          && (*current)[3] == 'L'
          && (*current)[4] == 'O'
          && (*current)[5] == 'C'
          && (*current)[6] == '_')
        {
          result = &(*current)[7];

          /* Save current position for next visit.  */
          *position = ++current;

          break;
        }

      ++current;
    }

  return result;
}
#endif


#ifdef SHARED
// 不能使用 sbrk
static void *
__failing_morecore (ptrdiff_t d)
{
  return (void *) MORECORE_FAILURE;
}

extern struct dl_open_hook *_dl_open_hook;
libc_hidden_proto (_dl_open_hook);
#endif

// 初始化 ptmalloc，在第一次调用 malloc 时会进行初始化
// 俩操作，一个将当前线程的 thread_arena 设置为 main_arena，另一个就是初始化 malloc_par mp_
static void
ptmalloc_init (void)
{
  // 已经初始化过或者正在初始化，无需重复初始化
  if (__malloc_initialized >= 0)
    return;

  // 设置开始初始化
  __malloc_initialized = 0;

#ifdef SHARED
  /* In case this libc copy is in a non-default namespace, never use brk.
     Likewise if dlopened from statically linked program.  */
  Dl_info di;
  struct link_map *l;

  // libc 是通过 dlopen 打开的，此时不允许使用 sbrk 分配内存，不连续
  if (_dl_open_hook != NULL
      || (_dl_addr (ptmalloc_init, &di, &l, NULL) != 0
          && l->l_ns != LM_ID_BASE))
    __morecore = __failing_morecore;
#endif

  // 因为是第一次初始化，即第一次使用，让当前线程使用 main_arena
  thread_arena = &main_arena;

#if HAVE_TUNABLES
  /* Ensure initialization/consolidation and do it under a lock so that a
     thread attempting to use the arena in parallel waits on us till we
     finish.  */
  // 修改 malloc_par 时使用 main_arena 的锁
  __libc_lock_lock (main_arena.mutex);
  // 合并 main_arena 的 fastbins
  malloc_consolidate (&main_arena);

  TUNABLE_SET_VAL_WITH_CALLBACK (check, NULL, set_mallopt_check);
  TUNABLE_SET_VAL_WITH_CALLBACK (top_pad, NULL, set_top_pad);
  TUNABLE_SET_VAL_WITH_CALLBACK (perturb, NULL, set_perturb_byte);
  TUNABLE_SET_VAL_WITH_CALLBACK (mmap_threshold, NULL, set_mmap_threshold);
  TUNABLE_SET_VAL_WITH_CALLBACK (trim_threshold, NULL, set_trim_threshold);
  TUNABLE_SET_VAL_WITH_CALLBACK (mmap_max, NULL, set_mmaps_max);
  TUNABLE_SET_VAL_WITH_CALLBACK (arena_max, NULL, set_arena_max);
  TUNABLE_SET_VAL_WITH_CALLBACK (arena_test, NULL, set_arena_test);
  __libc_lock_unlock (main_arena.mutex);
#else
  const char *s = NULL;
  // 从环境变量中解析 malloc_par
  if (__glibc_likely (_environ != NULL))
    {
      char **runp = _environ;
      char *envline;

      while (__builtin_expect ((envline = next_env_entry (&runp)) != NULL,
                               0))
        {
          // 获得 envline 中不包含 = 的最长前缀长度
          size_t len = strcspn (envline, "=");

          if (envline[len] != '=')
            /* This is a "MALLOC_" variable at the end of the string
               without a '=' character.  Ignore it since otherwise we
               will access invalid memory below.  */
            continue;
          // 根据环境变量设置 malloc_par,未开启 __libc_enable_secure 时才允许设置
          switch (len)
            {
            case 6:
              if (memcmp (envline, "CHECK_", 6) == 0)
                s = &envline[7];
              break;
            case 8:
              if (!__builtin_expect (__libc_enable_secure, 0))
                {
                  if (memcmp (envline, "TOP_PAD_", 8) == 0)
                    __libc_mallopt (M_TOP_PAD, atoi (&envline[9]));
                  else if (memcmp (envline, "PERTURB_", 8) == 0)
                    __libc_mallopt (M_PERTURB, atoi (&envline[9]));
                }
              break;
            case 9:
              if (!__builtin_expect (__libc_enable_secure, 0))
                {
                  if (memcmp (envline, "MMAP_MAX_", 9) == 0)
                    __libc_mallopt (M_MMAP_MAX, atoi (&envline[10]));
                  else if (memcmp (envline, "ARENA_MAX", 9) == 0)
                    __libc_mallopt (M_ARENA_MAX, atoi (&envline[10]));
                }
              break;
            case 10:
              if (!__builtin_expect (__libc_enable_secure, 0))
                {
                  if (memcmp (envline, "ARENA_TEST", 10) == 0)
                    __libc_mallopt (M_ARENA_TEST, atoi (&envline[11]));
                }
              break;
            case 15:
              if (!__builtin_expect (__libc_enable_secure, 0))
                {
                  if (memcmp (envline, "TRIM_THRESHOLD_", 15) == 0)
                    __libc_mallopt (M_TRIM_THRESHOLD, atoi (&envline[16]));
                  else if (memcmp (envline, "MMAP_THRESHOLD_", 15) == 0)
                    __libc_mallopt (M_MMAP_THRESHOLD, atoi (&envline[16]));
                }
              break;
            default:
              break;
            }
        }
    }
  if (s && s[0])
    {
      __libc_mallopt (M_CHECK_ACTION, (int) (s[0] - '0'));
      if (check_action != 0)
        __malloc_check_init ();
    }
#endif

#if HAVE_MALLOC_INIT_HOOK
  void (*hook) (void) = atomic_forced_read (__malloc_initialize_hook);
  if (hook != NULL)
    (*hook)();
#endif
  // 设置初始化完成
  __malloc_initialized = 1;
}

/* Managing heaps and arenas (for concurrent threads) */

#if MALLOC_DEBUG > 1

/* Print the complete contents of a single heap to stderr. */

static void
dump_heap (heap_info *heap)
{
  char *ptr;
  mchunkptr p;

  fprintf (stderr, "Heap %p, size %10lx:\n", heap, (long) heap->size);
  ptr = (heap->ar_ptr != (mstate) (heap + 1)) ?
        (char *) (heap + 1) : (char *) (heap + 1) + sizeof (struct malloc_state);
  p = (mchunkptr) (((unsigned long) ptr + MALLOC_ALIGN_MASK) &
                   ~MALLOC_ALIGN_MASK);
  for (;; )
    {
      fprintf (stderr, "chunk %p size %10lx", p, (long) p->size);
      if (p == top (heap->ar_ptr))
        {
          fprintf (stderr, " (top)\n");
          break;
        }
      else if (p->size == (0 | PREV_INUSE))
        {
          fprintf (stderr, " (fence)\n");
          break;
        }
      fprintf (stderr, "\n");
      p = next_chunk (p);
    }
}
#endif /* MALLOC_DEBUG > 1 */

/* If consecutive mmap (0, HEAP_MAX_SIZE << 1, ...) calls return decreasing
   addresses as opposed to increasing, new_heap would badly fragment the
   address space.  In that case remember the second HEAP_MAX_SIZE part
   aligned to HEAP_MAX_SIZE from last mmap (0, HEAP_MAX_SIZE << 1, ...)
   call (if it is already aligned) and try to reuse it next time.  We need
   no locking for it, as kernel ensures the atomicity for us - worst case
   we'll call mmap (addr, HEAP_MAX_SIZE, ...) for some value of addr in
   multiple threads, but only one will succeed.  */
static char *aligned_heap_area;

/* Create a new heap.  size is automatically rounded up to a multiple
   of the page size. */

// 创建一个新的 heap，一次 mmap HEAP_MAX_SIZE，将 size 大小设置为可读可写
// mmap 的映射需要保证按 HEAP_MAX_SIZE 对齐，因为根据 ptr 获得 heap 直接去掉非对齐部分得到 heap_info
static heap_info *
internal_function
new_heap (size_t size, size_t top_pad)
{
  // 获得 page 的大小
  size_t pagesize = GLRO (dl_pagesize);
  char *p1, *p2;
  unsigned long ul;
  heap_info *h;

  // 将需要的大小跟 heap 最小大小比较
  // 即需要申请的大小小于 HEAP_MIN_SIZE 时，按 HEAP_MIN_SIZE 申请
  // 大于 HEAP_MIN_SIZE 时，加上 pad 小于HEAP_MAX_SIZE，按 size+pad 申请
  // 加上 pad 大于 HEAP_MAX_SIZE 且本身没超过 HEAP_MAX_SIZE 时，按 HEAP_MAX_SIZE 申请
  // size 仅为设置可读可写的大小，实际申请大小为 HEAP_MAX_SIZE

  // 即 64 位上  size + pad < 32K,按32K申请
  // 32K < size + pad < 64M,按64M申请
  if (size + top_pad < HEAP_MIN_SIZE)
    size = HEAP_MIN_SIZE;
  else if (size + top_pad <= HEAP_MAX_SIZE)
    size += top_pad;
  else if (size > HEAP_MAX_SIZE)
    return 0;
  else
    size = HEAP_MAX_SIZE;
  // 申请大小按 page 对齐
  size = ALIGN_UP (size, pagesize);

  /* A memory region aligned to a multiple of HEAP_MAX_SIZE is needed.
     No swap space needs to be reserved for the following large
     mapping (on Linux, this is the case for all non-writable mappings
     anyway). */
  p2 = MAP_FAILED;
  // aligned_heap_area 为上次满足对齐时剩下的部分，尝试仍然从改地址映射，看能否 mmap HEAP_MAX_SIZE 大小
  if (aligned_heap_area)
    {
      p2 = (char *) MMAP (aligned_heap_area, HEAP_MAX_SIZE, PROT_NONE,
                          MAP_NORESERVE);
      aligned_heap_area = NULL;
      if (p2 != MAP_FAILED && ((unsigned long) p2 & (HEAP_MAX_SIZE - 1)))
        {
          // 映射成功但是没有对齐，直接释放
          __munmap (p2, HEAP_MAX_SIZE);
          p2 = MAP_FAILED;
        }
    }
  if (p2 == MAP_FAILED)
    {
      // 在最坏可能情况下，需要映射2倍HEAP_MAX_SIZE大小的虚拟内存才能实现地址按照HEAP_MAX_SIZE大小对齐
      p1 = (char *) MMAP (0, HEAP_MAX_SIZE << 1, PROT_NONE, MAP_NORESERVE);
      if (p1 != MAP_FAILED)
        {
          // 映射2倍HEAP_MAX_SIZE大小的虚拟内存成功，将大于等于p1并按HEAP_MAX_SIZE大小对齐的第一个虚拟地址赋值给p2，p2作为sub_heap的起始虚拟地址，
          // p2+ HEAP_MAX_SIZE作为sub_heap的结束地址，并将sub_heap的结束地址赋值给全局变量aligned_heap_area，最后还需要将多余的虚拟内存还回给操作系统
          p2 = (char *) (((unsigned long) p1 + (HEAP_MAX_SIZE - 1))
                         & ~(HEAP_MAX_SIZE - 1));
          ul = p2 - p1;
          // ul 为 heap 起始跟 分配起始的差值
          if (ul)
            // heap 起始不在分配起始，将前面部分还回去
            __munmap (p1, ul);
          else
            // heap 起始在分配其实，那么后面部分此次用不上
            // 将后面部分起始地址保存到 aligned_heap_area，并还回去，改地址实际上已按 HEAP_MAX_SIZE 对齐，下次分配尝试从改地址 mmap，这样可以保正连续性
            aligned_heap_area = p2 + HEAP_MAX_SIZE;
          __munmap (p2 + HEAP_MAX_SIZE, HEAP_MAX_SIZE - ul);
        }
      else
        {
          // 映射2倍HEAP_MAX_SIZE大小的虚拟内存失败
          /* Try to take the chance that an allocation of only HEAP_MAX_SIZE
             is already aligned. */
          // 尝试仅 mmap HEAP_MAX_SIZE 大小，如果能按 HEAP_MAX_SIZE 对齐就使用
          p2 = (char *) MMAP (0, HEAP_MAX_SIZE, PROT_NONE, MAP_NORESERVE);
          if (p2 == MAP_FAILED)
            return 0;

          if ((unsigned long) p2 & (HEAP_MAX_SIZE - 1))
            {
              // 没对齐，分配失败了
              __munmap (p2, HEAP_MAX_SIZE);
              return 0;
            }
        }
    }
  
  // 至此，HEAP_MAX_SIZE 大小且按 HEAP_MAX_SIZE 对齐的 heap 已分配成功
  // 将 heap 起始 size 大小设置为可读可写，因为后面要写入 heap_info 还有分配出去的大小
  if (__mprotect (p2, size, PROT_READ | PROT_WRITE) != 0)
    {
      __munmap (p2, HEAP_MAX_SIZE);
      return 0;
    }
  h = (heap_info *) p2;
  // 设置当前 heap 分配出去的大小
  h->size = size;
  // 设置当前 heap 设置可读可写的大小
  h->mprotect_size = size;
  LIBC_PROBE (memory_heap_new, 2, h, h->size);
  return h;
}

/* Grow a heap.  size is automatically rounded up to a
   multiple of the page size. */
// 将当前 heap 分配出去的内存增加 diff，实际是设置读写属性
static int
grow_heap (heap_info *h, long diff)
{
  size_t pagesize = GLRO (dl_pagesize);
  long new_size;
  // 将 diff 按 pagesize 向上对齐
  diff = ALIGN_UP (diff, pagesize);
  // 重新计算当前 heap 将要分配出去的大小
  new_size = (long) h->size + diff;
  // 当前要分配出去的大小已经超过 HEAP_MAX_SIZE 了
  if ((unsigned long) new_size > (unsigned long) HEAP_MAX_SIZE)
    return -1;
  // 当前要分配出去的大小超过了之前设置为可读可写的大小
  if ((unsigned long) new_size > h->mprotect_size)
    {
      // 将超过的部分设置可读可写属性
      if (__mprotect ((char *) h + h->mprotect_size,
                      (unsigned long) new_size - h->mprotect_size,
                      PROT_READ | PROT_WRITE) != 0)
        return -2;
      // 更新 mprotect_size
      h->mprotect_size = new_size;
    }
  // 更新分配使用的大小
  h->size = new_size;
  LIBC_PROBE (memory_heap_more, 2, h, h->size);
  return 0;
}

/* Shrink a heap.  */
// 将当前 heap 分配出去的内存减少 diff，实际是设置读写属性
static int
shrink_heap (heap_info *h, long diff)
{
  long new_size;
  // 重新计算当前 heap 将要分配出去的大小
  new_size = (long) h->size - diff;
  // 减小后的大小已经比 heap_info 部分大小还小了，不能减小
  if (new_size < (long) sizeof (*h))
    return -1;

  /* Try to re-map the extra heap space freshly to save memory, and make it
     inaccessible.  See malloc-sysdep.h to know when this is true.  */
  if (__glibc_unlikely (check_may_shrink_heap ()))
    {
      // 将减小 diff 后多出来的部分通过 mmap 设置为 PROT_NONE，不会占用实际的物理内存
      if ((char *) MMAP ((char *) h + new_size, diff, PROT_NONE,
                         MAP_FIXED) == (char *) MAP_FAILED)
        return -2;
      // 更新设置为可读可写的大小
      h->mprotect_size = new_size;
    }
  else
    __madvise ((char *) h + new_size, diff, MADV_DONTNEED);
  /*fprintf(stderr, "shrink %p %08lx\n", h, new_size);*/
  // 更新分配出去的大小
  h->size = new_size;
  LIBC_PROBE (memory_heap_less, 2, h, h->size);
  return 0;
}

/* Delete a heap. */
// 删除 heap
#define delete_heap(heap) \
  do {									      \
      if ((char *) (heap) + HEAP_MAX_SIZE == aligned_heap_area)		      \
        aligned_heap_area = NULL;					      \
      // 直接 unmap 当前 heap
      __munmap ((char *) (heap), HEAP_MAX_SIZE);			      \
    } while (0)

static int
internal_function
heap_trim (heap_info *heap, size_t pad)
{
  // 获得当前 heap 所在的 malloc_state
  mstate ar_ptr = heap->ar_ptr;
  unsigned long pagesz = GLRO (dl_pagesize);
  mchunkptr top_chunk = top (ar_ptr), p, bck, fwd;
  heap_info *prev_heap;
  long new_size, top_size, top_area, extra, prev_size, misalign;

  /* Can this heap go away completely? */
  // 判断当前 heap 能否完全被删除，因为 arena 没有删除操作，因此只有非 malloc_state 所在的 heap 才能被完全删除
  while (top_chunk == chunk_at_offset (heap, sizeof (*heap)))
    {
      // 获得当前 heap 的前一个 heap
      prev_heap = heap->prev;
      prev_size = prev_heap->size - (MINSIZE - 2 * SIZE_SZ);
      p = chunk_at_offset (prev_heap, prev_size);
      /* fencepost must be properly aligned.  */
      misalign = ((long) p) & MALLOC_ALIGN_MASK;
      p = chunk_at_offset (prev_heap, prev_size - misalign);
      assert (chunksize_nomask (p) == (0 | PREV_INUSE)); /* must be fencepost */
      p = prev_chunk (p);
      new_size = chunksize (p) + (MINSIZE - 2 * SIZE_SZ) + misalign;
      assert (new_size > 0 && new_size < (long) (2 * MINSIZE));
      if (!prev_inuse (p))
        new_size += prev_size (p);
      assert (new_size > 0 && new_size < HEAP_MAX_SIZE);
      if (new_size + (HEAP_MAX_SIZE - prev_heap->size) < pad + MINSIZE + pagesz)
        break;
      ar_ptr->system_mem -= heap->size;
      LIBC_PROBE (memory_heap_free, 2, heap, heap->size);
      delete_heap (heap);
      heap = prev_heap;
      if (!prev_inuse (p)) /* consolidate backward */
        {
          p = prev_chunk (p);
          unlink (ar_ptr, p, bck, fwd);
        }
      assert (((unsigned long) ((char *) p + new_size) & (pagesz - 1)) == 0);
      assert (((char *) p + new_size) == ((char *) heap + heap->size));
      top (ar_ptr) = top_chunk = p;
      set_head (top_chunk, new_size | PREV_INUSE);
      /*check_chunk(ar_ptr, top_chunk);*/
    }

  /* Uses similar logic for per-thread arenas as the main arena with systrim
     and _int_free by preserving the top pad and rounding down to the nearest
     page.  */
  top_size = chunksize (top_chunk);
  if ((unsigned long)(top_size) <
      (unsigned long)(mp_.trim_threshold))
    return 0;

  top_area = top_size - MINSIZE - 1;
  if (top_area < 0 || (size_t) top_area <= pad)
    return 0;

  /* Release in pagesize units and round down to the nearest page.  */
  extra = ALIGN_DOWN(top_area - pad, pagesz);
  if (extra == 0)
    return 0;

  /* Try to shrink. */
  if (shrink_heap (heap, extra) != 0)
    return 0;

  ar_ptr->system_mem -= extra;

  /* Success. Adjust top accordingly. */
  set_head (top_chunk, (top_size - extra) | PREV_INUSE);
  /*check_chunk(ar_ptr, top_chunk);*/
  return 1;
}

/* Create a new arena with initial size "size".  */

/* If REPLACED_ARENA is not NULL, detach it from this thread.  Must be
   called while free_list_lock is held.  */
// 取消线程对 replaced_arena 的使用 即 attach_threads 更新
static void
detach_arena (mstate replaced_arena)
{
  if (replaced_arena != NULL)
    {
      assert (replaced_arena->attached_threads > 0);
      /* The current implementation only detaches from main_arena in
	 case of allocation failure.  This means that it is likely not
	 beneficial to put the arena on free_list even if the
	 reference count reaches zero.  */
      --replaced_arena->attached_threads;
    }
}

// 创建一个新的 arena
static mstate
_int_new_arena (size_t size)
{
  mstate a;
  heap_info *h;
  char *ptr;
  unsigned long misalign;

  // 新建一个非主分配区，heap 需要包含一个 heap_info 和一个 malloc_state(arena 的第一个 heap 需要包含)
  h = new_heap (size + (sizeof (*h) + sizeof (*a) + MALLOC_ALIGNMENT),
                mp_.top_pad);
  if (!h)
    {
      /* Maybe size is too large to fit in a single heap.  So, just try
         to create a minimally-sized arena and let _int_malloc() attempt
         to deal with the large request via mmap_chunk().  */
      // 已经无法满足分配了，创建一个最小大小的 heap，仅包含一个 heap_info 和 malloc_state 的大小
      // 后续分配直接走 mmap
      h = new_heap (sizeof (*h) + sizeof (*a) + MALLOC_ALIGNMENT, mp_.top_pad);
      if (!h)
        return 0;
    }
  // 获得当前 arena 的 malloc_state，h + 1 按 c 语言数组操作即为 h + 1 * sizeof(*h)，即加上 heap_info 的大小
  a = h->ar_ptr = (mstate) (h + 1);
  // 初始化 malloc_state
  malloc_init_state (a);
  // 当前 arean 使用的线程数加 1
  a->attached_threads = 1;
  /*a->next = NULL;*/
  // 设置当前 arena 已经申请的内存大小为 heap 的大小
  a->system_mem = a->max_system_mem = h->size;

  /* Set up the top chunk, with proper alignment. */
  // a + 1 按 c 语言数组操作即为 a + 1 * sizeof(*a)，即加上 malloc_state 的大小，实际获得 malloc_state 后的起始地址
  ptr = (char *) (a + 1);
  // 将剩余部分的内存按 MALLOC_ALIGNMENT 对齐
  misalign = (unsigned long) chunk2mem (ptr) & MALLOC_ALIGN_MASK;
  if (misalign > 0)
    ptr += MALLOC_ALIGNMENT - misalign;
  // 让 arena 的 top chunk 指向 heap 去除 heap_info 和 malloc_state 后的大小
  top (a) = (mchunkptr) ptr;
  // 设置 top chunk 的内存头，即 size 并将 top chunk 前一个 chunk 设置为 INUSE
  set_head (top (a), (((char *) h + h->size) - ptr) | PREV_INUSE);

  LIBC_PROBE (memory_arena_new, 2, a, size);
  mstate replaced_arena = thread_arena;
  // 设置当前线程使用的 arena 为新分配的 a
  thread_arena = a;
  // 初始化 a 的锁
  __libc_lock_init (a->mutex);

  // 开始将 a 加到 arena 链中
  __libc_lock_lock (list_lock);

  /* Add the new arena to the global list.  */
  // 将 新建的 a 插入到 main_arena 后面，main_arena 是静态定义的
  a->next = main_arena.next;
  /* FIXME: The barrier is an attempt to synchronize with read access
     in reused_arena, which does not acquire list_lock while
     traversing the list.  */
  // 涉及到加链操作，确保上面的赋值先执行完成
  atomic_write_barrier ();
  main_arena.next = a;

  __libc_lock_unlock (list_lock);

  // 取消当前线程对之前所用的 arena 的使用
  __libc_lock_lock (free_list_lock);
  detach_arena (replaced_arena);
  __libc_lock_unlock (free_list_lock);

  /* Lock this arena.  NB: Another thread may have been attached to
     this arena because the arena is now accessible from the
     main_arena.next list and could have been picked by reused_arena.
     This can only happen for the last arena created (before the arena
     limit is reached).  At this point, some arena has to be attached
     to two threads.  We could acquire the arena lock before list_lock
     to make it less likely that reused_arena picks this new arena,
     but this could result in a deadlock with
     __malloc_fork_lock_parent.  */
  // 当前 a 已经加入到 arena 链中，防止其他线程 reused 当前 arena
  __libc_lock_lock (a->mutex);

  return a;
}


/* Remove an arena from free_list.  */
// 从 free list 中给当前线程获得一个空闲 arena，并将当前线程使用的 arena(如果存在) detach
static mstate
get_free_list (void)
{
  mstate replaced_arena = thread_arena;
  // free list 执行空闲的 arena 链表
  mstate result = free_list;
  // 空闲链表不为空
  if (result != NULL)
    {
      // 获得 free_list_lock
      __libc_lock_lock (free_list_lock);
      // 直接获得 free list 中第一个 arena
      result = free_list;
      if (result != NULL)
	{
    // 从 free list 中摘掉 result
	  free_list = result->next_free;

	  /* The arena will be attached to this thread.  */
	  assert (result->attached_threads == 0);
    // 让当前线程使用 result
	  result->attached_threads = 1;
    // 当前线程不再使用 replaced_arena，非空时更新其 attached_threads
	  detach_arena (replaced_arena);
	}
      __libc_lock_unlock (free_list_lock);

      // 成功获得 result
      if (result != NULL)
        {
          LIBC_PROBE (memory_arena_reuse_free_list, 1, result);
          // 当前线程使用 result，上锁
          __libc_lock_lock (result->mutex);
          // 更新当前线程的 thread_arena
	  thread_arena = result;
        }
    }

  return result;
}

/* Remove the arena from the free list (if it is present).
   free_list_lock must have been acquired by the caller.  */
static void
remove_from_free_list (mstate arena)
{
  mstate *previous = &free_list;
  for (mstate p = free_list; p != NULL; p = p->next_free)
    {
      assert (p->attached_threads == 0);
      if (p == arena)
	{
	  /* Remove the requested arena from the list.  */
	  *previous = p->next_free;
	  break;
	}
      else
	previous = &p->next_free;
    }
}

/* Lock and return an arena that can be reused for memory allocation.
   Avoid AVOID_ARENA as we have already failed to allocate memory in
   it and it is currently locked.  */
// 复用一个除了 avoid_arena(我们已经从里面分不出来内存了) 以外的 arena
static mstate
reused_arena (mstate avoid_arena)
{
  mstate result;
  /* FIXME: Access to next_to_use suffers from data races.  */
  static mstate next_to_use;
  // 按顺序复用所有 arena，从 main_arena 开始
  if (next_to_use == NULL)
    next_to_use = &main_arena;

  /* Iterate over all arenas (including those linked from
     free_list).  */
  result = next_to_use;
  do
    {
      // 为了避免 _int_new_arena 刚创建的 arena 被别的线程在这里被复用了，所以在 _int_new_arena 中先 lock 了
      // next_to_use 没有异常且能够上锁，说明这会儿没有人在用，开始复用该 arena
      if (!arena_is_corrupt (result) && !__libc_lock_trylock (result->mutex))
        goto out;

      /* FIXME: This is a data race, see _int_new_arena.  */
      // 遍历所有 arena
      result = result->next;
    }
    // arena 链是环
  while (result != next_to_use);

  /* Avoid AVOID_ARENA as we have already failed to allocate memory
     in that arena and it is currently locked.   */
  // 至此我们无法获得一个 arena，要么异常要么拿不到锁
  // 如果是 avoid_arena，换下一个
  if (result == avoid_arena)
    result = result->next;

  /* Make sure that the arena we get is not corrupted.  */
  mstate begin = result;
  // 遍历所有的 arena，avoid_arena 是已经分配不出来内存的 arena
  // 找到第一个不是异常或者 avoid_arena 的 arena
  while (arena_is_corrupt (result) || result == avoid_arena)
    {
      result = result->next;
      // 全部遍历完了没找到一个能用的，已经无法满足分配了
      if (result == begin)
	/* We looped around the arena list.  We could not find any
	   arena that was either not corrupted or not the one we
	   wanted to avoid.  */
	return NULL;
    }

  /* No arena available without contention.  Wait for the next in line.  */
  LIBC_PROBE (memory_arena_reuse_wait, 3, &result->mutex, result, avoid_arena);
  // 到这人 result 是第一个没异常的 arena，上面循环没 goto out 仅仅因为拿不到锁
  // 这会儿没有没被使用的 arena，直接等待拿锁，等其他线程用完
  __libc_lock_lock (result->mutex);

out:
  /* Attach the arena to the current thread.  */
  {
    // 至此我们已经拿到了将要使用的 arena 的锁，更新 thread_arena
    /* Update the arena thread attachment counters.   */
    mstate replaced_arena = thread_arena;
    __libc_lock_lock (free_list_lock);
    detach_arena (replaced_arena);

    /* We may have picked up an arena on the free list.  We need to
       preserve the invariant that no arena on the free list has a
       positive attached_threads counter (otherwise,
       arena_thread_freeres cannot use the counter to determine if the
       arena needs to be put on the free list).  We unconditionally
       remove the selected arena from the free list.  The caller of
       reused_arena checked the free list and observed it to be empty,
       so the list is very short.  */
    // 当前我们直接从 main_arena 后面的链里拿到新 arena，可能在 free list 中，摘链
    remove_from_free_list (result);

    // 增加 result 的使用
    ++result->attached_threads;

    __libc_lock_unlock (free_list_lock);
  }

  LIBC_PROBE (memory_arena_reuse, 2, result, avoid_arena);
  // 更新当前线程使用的 arena
  thread_arena = result;
  // 更新下一个被复用的 arena
  next_to_use = result->next;

  return result;
}

// 尝试创建或者从 free arena list 中获得一个 arena
static mstate
internal_function
arena_get2 (size_t size, mstate avoid_arena)
{
  mstate a;

  // 当前进程能创建的 arena 数量的最大值
  static size_t narenas_limit;
  // 从 free list 中获得一个 free 的 arena
  a = get_free_list ();
  // free list 中不存在空闲的 arena
  if (a == NULL)
    {
      /* Nothing immediately available, so generate a new arena.  */
      // arena_max 和 arena_test 仅能设置一个
      // arena_max 限制了进程能创建的 arena 的最大数量
      // arena_test 在已创建的 arena 数量少于 arena_test 时不会更新 narenas_limit(为0)，创建时判断不超过 0 - 1，即不限制arena的创建
      // 在已创建的 arena 数量超过 arena_test 时，会更新  narenas_limit 为 8 * 核数
      if (narenas_limit == 0)
        {
          // 从 mp_ 中获得 arena 数量限制
          if (mp_.arena_max != 0)
            narenas_limit = mp_.arena_max;
          // narenas 为当前进程已经创建的 arena 数量，超过 arena_test 时
          else if (narenas > mp_.arena_test)
            {
              // 获得当前 cpu 的核数
              int n = __get_nprocs ();
              // 多核设备
              if (n >= 1)
                // 设置 narenas_limit 为 8*n
                narenas_limit = NARENAS_FROM_NCORES (n);
              else
                /* We have no information about the system.  Assume two
                   cores.  */
                // 假设 2 个核
                narenas_limit = NARENAS_FROM_NCORES (2);
            }
        }
    repeat:;
      // 获得当前已经创建过的 arena 数量
      size_t n = narenas;
      /* NB: the following depends on the fact that (size_t)0 - 1 is a
         very large number and that the underflow is OK.  If arena_max
         is set the value of arena_test is irrelevant.  If arena_test
         is set but narenas is not yet larger or equal to arena_test
         narenas_limit is 0.  There is no possibility for narenas to
         be too big for the test to always fail since there is not
         enough address space to create that many arenas.  */
      // 已经创建的 arena 数量 没有超过 narenas_limit，可以创建新的 arena
      if (__glibc_unlikely (n <= narenas_limit - 1))
        {
          // 将 narenas 原子的设置为 n + 1
          if (catomic_compare_and_exchange_bool_acq (&narenas, n + 1, n))
            goto repeat;
          // 创建一个新的 arena，加入 main_arena 的链中，设置当前线程使用该 arena
          a = _int_new_arena (size);
	  if (__glibc_unlikely (a == NULL))
            catomic_decrement (&narenas);
        }
      else
        // 复用一个已经有线程在使用的 arena，设置当前线程使用该 arena
        a = reused_arena (avoid_arena);
    }
  return a;
}

/* If we don't have the main arena, then maybe the failure is due to running
   out of mmapped areas, so we can try allocating on the main arena.
   Otherwise, it is likely that sbrk() has failed and there is still a chance
   to mmap(), so try one of the other arenas.  */
static mstate
arena_get_retry (mstate ar_ptr, size_t bytes)
{
  LIBC_PROBE (memory_arena_retry, 2, bytes, ar_ptr);
  if (ar_ptr != &main_arena)
    {
      __libc_lock_unlock (ar_ptr->mutex);
      /* Don't touch the main arena if it is corrupt.  */
      if (arena_is_corrupt (&main_arena))
	return NULL;

      ar_ptr = &main_arena;
      __libc_lock_lock (ar_ptr->mutex);
    }
  else
    {
      __libc_lock_unlock (ar_ptr->mutex);
      ar_ptr = arena_get2 (bytes, ar_ptr);
    }

  return ar_ptr;
}

static void __attribute__ ((section ("__libc_thread_freeres_fn")))
arena_thread_freeres (void)
{
  mstate a = thread_arena;
  thread_arena = NULL;

  if (a != NULL)
    {
      __libc_lock_lock (free_list_lock);
      /* If this was the last attached thread for this arena, put the
	 arena on the free list.  */
      assert (a->attached_threads > 0);
      if (--a->attached_threads == 0)
	{
	  a->next_free = free_list;
	  free_list = a;
	}
      __libc_lock_unlock (free_list_lock);
    }
}
text_set_element (__libc_thread_subfreeres, arena_thread_freeres);

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
