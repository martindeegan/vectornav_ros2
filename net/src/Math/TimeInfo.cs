using System;

namespace VectorNav.Math {

  /// <summary>
  /// TimeStatus and Leapseconds
  /// </summary>
  public struct TimeInfo {

    #region Public Members

    /// <summary>
    /// The TimeStatus value.
    /// </summary>
    public byte TimeStatus;
    /// <summary>
    /// The LeapSeconds value.
    /// </summary>
    public sbyte LeapSeconds;

    #endregion

    #region Properties

    /// <summary>
    /// The ValidTOW flag of the TimeStatus <see cref="TimeInfo"/>.
    /// </summary>
    public bool ValidTow { get { return (bool)((TimeStatus & 0x1) == 0x1); } }

    /// <summary>
    /// The ValidTOW flag of the TimeStatus <see cref="TimeInfo"/>.
    /// </summary>
    public bool ValidWkn { get { return (bool)((TimeStatus & 0x2) == 0x2); } }

    /// <summary>
    /// The ValidTOW flag of the TimeStatus <see cref="TimeInfo"/>.
    /// </summary>
    public bool ValidUtc { get { return (bool)((TimeStatus & 0x4) == 0x4); } }

    #endregion

    #region Constructors

    public TimeInfo(byte status, sbyte leapSecs) {
      TimeStatus = status;
      LeapSeconds = leapSecs;
    }

    #endregion

    #region Methods
    #endregion
  }

}