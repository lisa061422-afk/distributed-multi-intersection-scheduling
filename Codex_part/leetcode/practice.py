from math import gcd
from typing import List

# nums = [10, 20, 30]
# nums.append(4)
# nums.pop()
# nums[0]
# len(nums)

# for x in nums:
#     print(x)

# for i, x in enumerate(nums):
#     print(i,x)

# set 哈希集合：查有没有
# seen = set()

# seen.add(3)
# seen.add(5)
# print(seen)          # {3, 5}
# print(3 in seen)    # True
# print(4 in seen)    # False

# nums = [3, 5, 3, 2]
# for num in nums:
#     if num in seen:
#         print(num, "已经在 seen 里")
#     else:
#         print(num, "第一次见")
#         seen.add(num)

# dict 哈希表：记录 key -> value
# mp = {}

# mp["apple"] = 3
# mp["banana"] = 5

# print(mp)              # {'apple': 3, 'banana': 5}
# print(mp["apple"])     # 3

# print("apple" in mp)   # True
# print("orange" in mp)  # False

# def twoSum(nums, target): # this is input of a function
#     mp = {}
    
#     for i, num in enumerate(nums):
#         need = target - num

#         if need in mp:
#             return [mp[need], i]
        
#         mp[num] = i

# nums = [3, 4, 5, 6]
# target = 7

# print(twoSum(nums, target))  # [0, 1]

# def encode(strs: list[str]) -> str:
#         parts = []
#         for s in strs:
#             parts.append(str(len(s)))
#             parts.append("#")
#             parts.append(s) 

#         return "".join(parts)

# def decode(s: str) -> list[str]:
#         res = []
#         i = 0 #指针起点

#         while i < len(s):
#             j = i #初始化指针终点

#             while s[j] != "#":
#                 j += 1
            
#             length = int(s[i:j]) #[start,end) 不包括end这个位置的
#             start = j + 1
#             end = start + length

#             res.append(s[start:end])

#             i = end
#         return res

# x = encode(["hello","world"])
# print(x)
# y = decode(x)
# print(y)

# def productExceptSelf(nums: List[int]) -> List[int]:
#         n = len(nums)
#         res = [1] * n

#         prefix = 1
#         for i in range(n):
#             res[i] = prefix
#             prefix *= nums[i]

#         postfix = 1
#         for i in range(n - 1, -1, -1):
#             res[i] *= postfix
#             postfix *= nums[i]

#         return res

# x = productExceptSelf([2,5,8])
# print(x)

# x = 1 // 3
# x.add("1")
# x = [set() for _ in range(9)]
# word1 = "abc"
# str1 = "abc"
# str2 = 'cba'

# print(''.join([str1, str2]))
# print(str1 + str2)

# nums = [1,2,4,3]
# nums.append(4)
# nums.insert(5,10)
# nums.remove(2) # remove the first occurrence of value 2
# nums.pop() # remove the last element
# nums.pop(2) #删除index is 0的元素

# print(6 in nums)

# def decodeString(s: str) -> str:
#     string_stack = []
#     number_stack = []

#     current_string = ""
#     current_number = 0

#     for char in s:
#         if char.isdigit():
#             current_number = current_number * 10 + int(char)

#         elif char == '[':
#             number_stack.append(current_number)
#             string_stack.append(current_string)

#             current_number = 0
#             current_string = ""

#         elif char == ']':
#             repeat_number = number_stack.pop()
#             previous_string = string_stack.pop()

#             current_string = (
#                 previous_string +
#                 current_string * repeat_number
#             )

#         else:
#             current_string += char

#     return current_string


# x = decodeString("3[a2[c]]")
# print(x) 

from collections import deque


class RecentCounter:

    def __init__(self):
        # 创建一个空队列，用来保存每次请求的时间
        self.queue = deque()

    def ping(self, t: int) -> int:
        # 把本次请求时间加入队列
        self.queue.append(t)

        # 删除所有早于 t - 3000 的请求
        while self.queue[0] < t - 3000:
            self.queue.popleft()

        # 队列中剩余的元素数量，就是最近 3000 ms 的请求次数
        return len(self.queue)


# 创建一个 RecentCounter 对象
recentCounter = RecentCounter()

# 依次调用同一个对象的 ping 方法
print(recentCounter.ping(1))      # 输出：1
print(recentCounter.ping(100))    # 输出：2
print(recentCounter.ping(3001))   # 输出：3
print(recentCounter.ping(3002))   # 输出：3