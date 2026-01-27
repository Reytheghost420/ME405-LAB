
=== DEMO START ===

[TEST 2] Enable motors: should NOT move when enabled (effort=0)

After enable, effort=0 (positions should stay near 0)
L pos: 0  | R pos: 0
L pos: 0  | R pos: 0
L pos: 0  | R pos: 0
L pos: 0  | R pos: 0
L pos: 0  | R pos: 0
L pos: 0  | R pos: 0
L pos: 0  | R pos: 0
L pos: 0  | R pos: 0

[TEST 1] Right motor only, forward speeds 30 -> 60

Right +30 only (R should change, L ~0)
L pos: 0  | R pos: 0
L pos: 335  | R pos: 0
L pos: 742  | R pos: 0
L pos: 1152  | R pos: 0
L pos: 1562  | R pos: 0
L pos: 1974  | R pos: 0
L pos: 2385  | R pos: 0
L pos: 2797  | R pos: 0
L pos: 3209  | R pos: 0
L pos: 3621  | R pos: 0

Right +60 only (R should change faster)
L pos: 4042  | R pos: 0
L pos: 4770  | R pos: 0
L pos: 5540  | R pos: 0
L pos: 6308  | R pos: 0
L pos: 7073  | R pos: 0
L pos: 7843  | R pos: 0
L pos: 8614  | R pos: 0
L pos: 9385  | R pos: 0
L pos: 10153  | R pos: 0
L pos: 10924  | R pos: 0

[TEST 1] Left motor only, backward speeds -30 -> -60

Left -30 only (L should change, R ~0)
L pos: 15  | R pos: 0
L pos: 783  | R pos: -306
L pos: 1550  | R pos: -685
L pos: 2316  | R pos: -1068
L pos: 3085  | R pos: -1452
L pos: 3855  | R pos: -1838
L pos: 4627  | R pos: -2223
L pos: 5400  | R pos: -2607
L pos: 6173  | R pos: -2991
L pos: 6947  | R pos: -3375

Left -60 only (L should change faster)
L pos: 7737  | R pos: -3767
L pos: 8506  | R pos: -4491
L pos: 9278  | R pos: -5306
L pos: 10053  | R pos: -6130
L pos: 10827  | R pos: -6954
L pos: 11600  | R pos: -7776
L pos: 12372  | R pos: -8598
L pos: 13144  | R pos: -9419
L pos: 13918  | R pos: -10240
L pos: 14692  | R pos: -11061

[TEST 3] Both motors forward then backward (positions should go + then -)
Both +40 (positions should increase)
L pos: 15  | R pos: -16
L pos: 626  | R pos: -633
L pos: 1194  | R pos: -1168
L pos: 1764  | R pos: -1705
L pos: 2333  | R pos: -2242
L pos: 2901  | R pos: -2779
L pos: 3469  | R pos: -3314
L pos: 4038  | R pos: -3851
L pos: 4606  | R pos: -4387
L pos: 5173  | R pos: -4923
L pos: 5741  | R pos: -5459
L pos: 6308  | R pos: -5995

Both -40 (positions should decrease or wrap)
L pos: 6886  | R pos: -6544
L pos: 6521  | R pos: -7073
L pos: 5960  | R pos: -7609
L pos: 5397  | R pos: -8145
L pos: 4836  | R pos: -8682
L pos: 4276  | R pos: -9218
L pos: 3716  | R pos: -9753
L pos: 3156  | R pos: -10289
L pos: 2597  | R pos: -10826
L pos: 2039  | R pos: -11362
L pos: 1481  | R pos: -11899
L pos: 923  | R pos: -12436

[TEST 4] Overflow/underflow handling (no crazy jump in position)

Run fast a moment
L pos: -6  | R pos: -7
L pos: 736  | R pos: -917
L pos: 1720  | R pos: -1925
L pos: 2703  | R pos: -2934
L pos: 3685  | R pos: -3938
L pos: 4662  | R pos: -4932
L pos: 5631  | R pos: -5921
L pos: 6597  | R pos: -6911
L pos: 7557  | R pos: -7907
L pos: 8516  | R pos: -8902

Reverse fast a moment (should still be smooth)
L pos: 9496  | R pos: -9920
L pos: 8834  | R pos: -10916
L pos: 7866  | R pos: -11936
L pos: 6909  | R pos: -12964
L pos: 5958  | R pos: -13994
L pos: 5014  | R pos: -15014
L pos: 4077  | R pos: -16034
L pos: 3142  | R pos: -17052
L pos: 2206  | R pos: -18071
L pos: 1272  | R pos: -19087

[TEST 5] Pairing check: spinning a motor should move its encoder

Left motor only: L should move, R ~0
L pos: -18  | R pos: -20
L pos: -951  | R pos: -804
L pos: -1884  | R pos: -1498
L pos: -2821  | R pos: -2192
L pos: -3759  | R pos: -2886
L pos: -4690  | R pos: -3579
L pos: -5614  | R pos: -4271
L pos: -6538  | R pos: -4962
L pos: -7471  | R pos: -5652
L pos: -8400  | R pos: -6341

Right motor only: R should move, L ~0
L pos: -9340  | R pos: -7043
L pos: -8935  | R pos: -7722
L pos: -8235  | R pos: -8410
L pos: -7547  | R pos: -9099
L pos: -6860  | R pos: -9789
L pos: -6175  | R pos: -10480
L pos: -5493  | R pos: -11167
L pos: -4812  | R pos: -11855
L pos: -4130  | R pos: -12544
L pos: -3449  | R pos: -13234

[TEST 6] Check sign convention: +effort should make forward-driving counts go up
If this is reversed, swap encoder A/B wires OR swap channel(1)/channel(2) in software.

Apply +40 (expect positions to increase for forward direction)
L pos: 21  | R pos: -21
L pos: 611  | R pos: -605
L pos: 1181  | R pos: -1158
L pos: 1746  | R pos: -1715
L pos: 2311  | R pos: -2273
L pos: 2873  | R pos: -2830
L pos: 3434  | R pos: -3387
L pos: 3993  | R pos: -3943
L pos: 4551  | R pos: -4499
L pos: 5110  | R pos: -5055

=== DEMO END ===
